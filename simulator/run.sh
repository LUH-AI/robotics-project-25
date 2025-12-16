xhost +local:

apptainer instance start --nv\
    --env PRIVACY_CONSENT=Y \
    --env DISPLAY=$DISPLAY \
    --env ACCEPT_EULA=Y \
    --bind /apps/conda:/apps/conda \
    --bind /apps/tmux:/apps/tmux \
    --bind /apps/compilers/cuda:/apps/compilers/cuda \
    container.sif isaacsim
