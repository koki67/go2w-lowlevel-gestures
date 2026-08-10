COMPOSE := docker compose
SERVICE := controller

.PHONY: build describe test preflight live clean

build:
	$(COMPOSE) build --pull

describe:
	$(COMPOSE) run --rm --no-deps -T $(SERVICE) --describe

test:
	$(COMPOSE) run --rm --no-deps -T \
		--entrypoint /opt/venv/bin/python $(SERVICE) \
		-m unittest discover -s /app/tests -v

preflight:
	$(COMPOSE) run --rm --no-deps $(SERVICE)

live:
	$(COMPOSE) run --rm --no-deps $(SERVICE) --live

clean:
	$(COMPOSE) down --remove-orphans
