CUDA_VISIBLE_DEVICES=1 vllm serve llava-hf/llava-1.5-7b-hf \
    --host 0.0.0.0 \
    --port 8000 \
    --trust-remote-code \
    --max-model-len 4096