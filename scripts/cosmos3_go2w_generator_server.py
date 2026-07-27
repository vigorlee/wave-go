#!/usr/bin/env python3
"""Start the persistent Cosmos3 action server with inference-only safeguards."""

from __future__ import annotations

import base64
from io import BytesIO
import json
from pathlib import Path
import threading
import traceback

from PIL import Image

from cosmos_framework.inference.args import OmniSetupOverrides
from cosmos_framework.scripts import action_policy_server_libero as action_server
from cosmos_framework.scripts.action_policy_server_utils import (
    DEFAULT_FALLBACK_OUTPUT_DIR,
)


def build_inference_only_setup(
    self: action_server.ActionServerArgs,
) -> OmniSetupOverrides:
    if not getattr(self.checkpoint, "checkpoint_path", ""):
        raise ValueError("--checkpoint-path is required")

    output_dir = self.output_dir or self.dump_dir or DEFAULT_FALLBACK_OUTPUT_DIR
    setup = OmniSetupOverrides.model_validate(self.checkpoint.model_dump())
    setup.output_dir = Path(output_dir)
    setup.sampler = self.sampler
    setup.guardrails = False
    setup.parallelism_preset = "latency"
    return setup


def install_shared_reasoner_endpoint() -> None:
    """Expose deterministic per-request seeds and the loaded Edge Reasoner."""

    original_get_info = action_server.ActionModelService.get_info
    original_predict_policy_batch = (
        action_server.ActionModelService.predict_policy_batch
    )

    def get_info_with_reasoner(
        self: action_server.ActionModelService,
    ) -> dict[str, object]:
        payload = original_get_info(self)
        payload["reasoner"] = True
        payload["reasoner_endpoint"] = "/reason"
        payload["request_seed_supported"] = True
        return payload

    def predict_policy_batch_with_seed(
        self: action_server.ActionModelService,
        requests: list[dict[str, object]],
    ) -> dict[str, object]:
        if not requests:
            return original_predict_policy_batch(self, requests)
        seeds = [request.get("seed", self.cfg.seed) for request in requests]
        if any(
            isinstance(seed, bool)
            or not isinstance(seed, int)
            or not 0 <= seed <= 2_147_483_647
            for seed in seeds
        ):
            raise ValueError("'seed' must be an integer in [0, 2147483647]")
        if len(set(seeds)) != 1:
            raise ValueError(
                "one /predict_batch request must use one shared seed; "
                "send distinct candidates as serialized requests"
            )
        seed_lock = getattr(self, "_request_seed_lock", None)
        if seed_lock is None:
            seed_lock = threading.Lock()
            self._request_seed_lock = seed_lock
        with seed_lock:
            previous_seed = self.cfg.seed
            object.__setattr__(self.cfg, "seed", seeds[0])
            try:
                return original_predict_policy_batch(self, requests)
            finally:
                object.__setattr__(self.cfg, "seed", previous_seed)

    def predict_reasoner(
        self: action_server.ActionModelService, request: dict[str, object]
    ) -> dict[str, str]:
        prompt = request.get("prompt")
        image_b64 = request.get("image")
        max_new_tokens = request.get("max_new_tokens", 512)
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("'prompt' must be a non-empty string")
        if not isinstance(image_b64, str):
            raise ValueError("'image' must be a base64 string")
        if (
            isinstance(max_new_tokens, bool)
            or not isinstance(max_new_tokens, int)
            or not 32 <= max_new_tokens <= 1024
        ):
            raise ValueError("'max_new_tokens' must be in [32, 1024]")
        if image_b64.startswith("data:"):
            image_b64 = image_b64.split(",", 1)[-1]
        try:
            image = Image.open(BytesIO(base64.b64decode(image_b64))).convert("RGB")
            image.load()
        except Exception as exc:
            raise ValueError(f"invalid reasoner image: {exc}") from exc
        with self._lock:
            texts = self.model.generate_reasoner_text(
                [prompt],
                max_new_tokens=max_new_tokens,
                images=[image],
                videos=None,
                do_sample=False,
                temperature=1.0,
                top_k=None,
                top_p=None,
                repetition_penalty=1.0,
                presence_penalty=0.0,
                seed=0,
            )
        if not isinstance(texts, list) or len(texts) != 1 or not texts[0].strip():
            raise RuntimeError("Reasoner returned no text")
        return {"reasoner_text": texts[0]}

    original_do_post = action_server._ActionHandler.do_POST

    def do_post_with_reasoner(self: action_server._ActionHandler) -> None:
        if self.path != "/reason":
            original_do_post(self)
            return
        content_type = (
            (self.headers.get("Content-Type") or "").split(";", 1)[0].strip().lower()
        )
        if content_type != "application/json":
            self._send_json(415, {"error": "Content-Type must be application/json"})
            return
        try:
            length = int(self.headers.get("Content-Length") or "0")
        except ValueError:
            self._send_json(400, {"error": "Invalid Content-Length"})
            return
        if not 0 < length <= 2_000_000:
            self._send_json(413, {"error": "Request body must be at most 2 MB"})
            return
        try:
            request = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("JSON body must be an object")
            service = getattr(self.server, "service")
            output = service.predict_reasoner(request)
        except Exception as exc:
            traceback.print_exc()
            action_server.log.error(f"[action-server] /reason ERROR: {exc}")
            self._send_json(400, {"error": str(exc)})
            return
        self._send_json(200, output)

    action_server.ActionModelService.get_info = get_info_with_reasoner
    action_server.ActionModelService.predict_policy_batch = (
        predict_policy_batch_with_seed
    )
    action_server.ActionModelService.predict_reasoner = predict_reasoner
    action_server._ActionHandler.do_POST = do_post_with_reasoner


def main() -> None:
    # The upstream action server does not expose its setup-level guardrail flag.
    # Robot policy requests use the local safety shield instead of text guardrails.
    action_server.ActionServerArgs.build_setup_overrides = build_inference_only_setup
    install_shared_reasoner_endpoint()
    action_server.main()


if __name__ == "__main__":
    main()
