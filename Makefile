IMAGE_NAME=har-cont
CONTAINER_NAME=$(IMAGE_NAME)

RUN_ARGS=--name=$(CONTAINER_NAME) \
				 --device=/dev/video0:/dev/video0 \
				 -p 5000:5000 -p 8000:8000 -p 42424:42424 -p 4200:4200 -p 8765:8765 --rm $(IMAGE_NAME)

.PHONY : build run run-old connect build-and-run

build:
	docker build -t $(IMAGE_NAME) .

run:
	set -e; \
	docker run -it --gpus all $(RUN_ARGS)

run-old:
	set -e; \
	docker run --runtime=nvidia $(RUN_ARGS)

connect:
	set -e; \
	docker exec -it $(CONTAINER_NAME) bash

pipeline:
	set -e; \
	docker exec -it $(CONTAINER_NAME) bash -i -c 'rosrun ha_recognition HAR_client.py'

ravestate:
	set -e; \
	docker exec -it $(CONTAINER_NAME) bash -i -c 'cd ravestate; python3 -m ravestate -f config/charades.yml'



build-and-run: build run
