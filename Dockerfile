FROM public.ecr.aws/paia-tech/pros-utils:20231121

RUN apt update && \
    apt install -y tmux ncdu

RUN pip install --no-cache-dir torch torchvision torchaudio

ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash","-l"]
