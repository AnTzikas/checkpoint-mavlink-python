# checkpoint-mavlink-python

<!-- --mount type=bind,src="$(pwd)/logfiles",dst=/app/host \ -->

docker run -d --name chk \
            --privileged \
            --network host \
            --security-opt seccomp=unconfined \
            --mount type=bind,src="$(pwd)",dst=/app \
            -w /app antzikas/mission-container:final \
            sleep infinity
