"""
Deep retrieval for reference-map chip selection.

This is benchmark-focused: reduce search space (retrieve top-K chips) before
running local matching / pose solve.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


def _sha1_of_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    h = hashlib.sha1()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk_size)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


@dataclass(frozen=True)
class Chip:
    chip_id: int
    x0: int
    y0: int
    w: int
    h: int
    center_px: Tuple[float, float]


@dataclass(frozen=True)
class RetrievalResult:
    chip: Chip
    score: float  # higher is better


class RetrievalBackend:
    """Abstract embedding backend."""

    def embed_bgr(self, img_bgr: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    @property
    def embedding_dim(self) -> int:
        raise NotImplementedError

    @property
    def name(self) -> str:
        return self.__class__.__name__


class TorchResNet18Backend(RetrievalBackend):
    """
    ResNet18 global image embedding (torch/torchvision).

    Notes:
      - Optional dependency: will raise ImportError if torch/torchvision missing.
      - Uses ImageNet pretrained weights; embedding is avgpool feature (512-d).
    """

    def __init__(self, device: str = "cpu"):
        try:
            import torch
            import torchvision
            from torchvision import transforms
        except Exception as e:  # pragma: no cover
            raise ImportError(
                "Deep retrieval backend requires torch + torchvision. "
                "Install them (CPU or CUDA) before running vps_avl_benchmark."
            ) from e

        self._torch = torch
        self._device = torch.device(device)

        weights = torchvision.models.ResNet18_Weights.DEFAULT
        model = torchvision.models.resnet18(weights=weights)
        # Remove classification head; keep avgpool features.
        model.fc = torch.nn.Identity()
        model.eval()
        model.to(self._device)

        self._model = model
        self._pre = transforms.Compose(
            [
                transforms.ToPILImage(),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(mean=weights.transforms().mean, std=weights.transforms().std),
            ]
        )

    @property
    def embedding_dim(self) -> int:
        return 512

    def embed_bgr(self, img_bgr: np.ndarray) -> np.ndarray:
        torch = self._torch
        rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        x = self._pre(rgb).unsqueeze(0).to(self._device)
        with torch.no_grad():
            y = self._model(x)
        emb = y.detach().cpu().numpy().astype(np.float32).reshape(-1)
        # L2 normalize for cosine similarity
        n = float(np.linalg.norm(emb) + 1e-12)
        return emb / n


class ReferenceChipIndex:
    """
    Tiles a reference image into chips and builds an embedding index.
    """

    def __init__(
        self,
        ref_bgr: np.ndarray,
        backend: RetrievalBackend,
        chip_size: int = 768,
        stride: int = 512,
    ):
        if ref_bgr is None or ref_bgr.size == 0:
            raise ValueError("Empty reference image")
        self.ref_bgr = ref_bgr
        self.backend = backend
        self.chip_size = int(chip_size)
        self.stride = int(stride)
        self.chips: List[Chip] = []
        self.embeddings: Optional[np.ndarray] = None  # (N, D)

    def _generate_chips(self) -> List[Chip]:
        h, w = self.ref_bgr.shape[:2]
        cs = self.chip_size
        st = self.stride
        chips: List[Chip] = []
        chip_id = 0
        for y0 in range(0, max(1, h - cs + 1), st):
            for x0 in range(0, max(1, w - cs + 1), st):
                x1 = min(w, x0 + cs)
                y1 = min(h, y0 + cs)
                ww = x1 - x0
                hh = y1 - y0
                cx = x0 + ww * 0.5
                cy = y0 + hh * 0.5
                chips.append(Chip(chip_id=chip_id, x0=x0, y0=y0, w=ww, h=hh, center_px=(cx, cy)))
                chip_id += 1
        # Ensure at least one chip (small refs)
        if not chips:
            chips = [Chip(chip_id=0, x0=0, y0=0, w=w, h=h, center_px=(w / 2, h / 2))]
        return chips

    def build(self) -> None:
        self.chips = self._generate_chips()
        D = self.backend.embedding_dim
        embs = np.zeros((len(self.chips), D), dtype=np.float32)
        for i, c in enumerate(self.chips):
            chip_img = self.ref_bgr[c.y0 : c.y0 + c.h, c.x0 : c.x0 + c.w]
            embs[i, :] = self.backend.embed_bgr(chip_img)
        self.embeddings = embs

    def save(self, path: Path, extra_meta: Optional[dict] = None) -> None:
        if self.embeddings is None:
            raise ValueError("Index not built")
        meta = {
            "backend": self.backend.name,
            "chip_size": self.chip_size,
            "stride": self.stride,
            "n_chips": len(self.chips),
        }
        if extra_meta:
            meta.update(extra_meta)
        chips_arr = np.array([[c.chip_id, c.x0, c.y0, c.w, c.h, c.center_px[0], c.center_px[1]] for c in self.chips],
                             dtype=np.float32)
        np.savez_compressed(path, embeddings=self.embeddings, chips=chips_arr, meta=json.dumps(meta))

    @classmethod
    def load(cls, path: Path, ref_bgr: np.ndarray, backend: RetrievalBackend) -> "ReferenceChipIndex":
        data = np.load(path, allow_pickle=False)
        meta = json.loads(str(data["meta"]))
        idx = cls(ref_bgr=ref_bgr, backend=backend,
                  chip_size=int(meta["chip_size"]), stride=int(meta["stride"]))
        chips_arr = data["chips"]
        chips: List[Chip] = []
        for row in chips_arr:
            chips.append(
                Chip(
                    chip_id=int(row[0]),
                    x0=int(row[1]),
                    y0=int(row[2]),
                    w=int(row[3]),
                    h=int(row[4]),
                    center_px=(float(row[5]), float(row[6])),
                )
            )
        idx.chips = chips
        idx.embeddings = data["embeddings"].astype(np.float32)
        return idx

    def query(self, query_bgr: np.ndarray, topk: int = 5) -> List[RetrievalResult]:
        if self.embeddings is None:
            raise ValueError("Index not built")
        q = self.backend.embed_bgr(query_bgr).reshape(1, -1)
        # cosine similarity (embeddings already normalized)
        scores = (self.embeddings @ q.T).reshape(-1)
        k = int(max(1, min(topk, len(scores))))
        idxs = np.argpartition(-scores, k - 1)[:k]
        idxs = idxs[np.argsort(-scores[idxs])]
        return [RetrievalResult(chip=self.chips[i], score=float(scores[i])) for i in idxs]


def default_cache_path(
    cache_dir: Path,
    ref_image_path: Path,
    backend: RetrievalBackend,
    chip_size: int,
    stride: int,
) -> Path:
    cache_dir.mkdir(parents=True, exist_ok=True)
    key = f"{_sha1_of_file(ref_image_path)}_{backend.name}_{chip_size}_{stride}"
    return cache_dir / f"retrieval_index_{key}.npz"

