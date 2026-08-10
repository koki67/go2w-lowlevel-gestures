COMPOSE := docker compose
SERVICE := controller

.PHONY: help build describe describe-height describe-roll test \
	preflight-height preflight-roll live-height live-roll clean

help:
	@echo "Non-hardware: make build | test | describe"
	@echo "Read-only:    make preflight-height | preflight-roll"
	@echo "Physical:     make live-height | live-roll"

build:
	$(COMPOSE) build --pull

describe:
	$(COMPOSE) run --rm --no-deps -T $(SERVICE) --describe

describe-height:
	$(COMPOSE) run --rm --no-deps -T $(SERVICE) --gesture height --describe

describe-roll:
	$(COMPOSE) run --rm --no-deps -T $(SERVICE) --gesture roll --describe

test:
	$(COMPOSE) run --rm --no-deps -T \
		--entrypoint /opt/venv/bin/python $(SERVICE) \
		-m unittest discover -s /app/tests -v

preflight-height:
	$(COMPOSE) run --rm --no-deps $(SERVICE) --gesture height

preflight-roll:
	$(COMPOSE) run --rm --no-deps $(SERVICE) --gesture roll

live-height:
	$(COMPOSE) run --rm --no-deps $(SERVICE) --gesture height --live

live-roll:
	$(COMPOSE) run --rm --no-deps $(SERVICE) --gesture roll --live

clean:
	$(COMPOSE) down --remove-orphans
