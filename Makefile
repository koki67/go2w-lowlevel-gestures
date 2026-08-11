COMPOSE := docker compose
SERVICE := controller
PYTHON := /opt/venv/bin/python
SLOW_SCRIPT := /app/go2w_gesture_real.py
FAST_SCRIPT := /app/go2w_gesture_real_fast.py

.PHONY: help build test clean describe \
	describe-slow describe-slow-height describe-slow-roll \
	describe-fast describe-fast-height describe-fast-roll \
	describe-height describe-roll \
	preflight-slow-height preflight-slow-roll \
	preflight-fast-height preflight-fast-roll \
	preflight-height preflight-roll \
	live-slow-height live-slow-roll \
	live-fast-height live-fast-roll \
	live-height live-roll

help:
	@echo "Non-hardware: make build | test | describe"
	@echo "Slow 2.0/2.0 read-only: make preflight-slow-height | preflight-slow-roll"
	@echo "Slow 2.0/2.0 physical:  make live-slow-height | live-slow-roll"
	@echo "Fast 1.0/0.5 read-only: make preflight-fast-height | preflight-fast-roll"
	@echo "Fast 1.0/0.5 physical:  make live-fast-height | live-fast-roll"
	@echo "Compatibility: preflight-height/roll and live-height/roll use slow"

build:
	$(COMPOSE) build --pull

describe: describe-slow describe-fast

describe-slow:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --describe

describe-slow-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture height --describe

describe-slow-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture roll --describe

describe-fast:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --describe

describe-fast-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --gesture height --describe

describe-fast-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --gesture roll --describe

describe-height: describe-slow-height

describe-roll: describe-slow-roll

test:
	$(COMPOSE) run --rm --no-deps -T \
		--entrypoint /opt/venv/bin/python $(SERVICE) \
		-m unittest discover -s /app/tests -v

preflight-slow-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture height

preflight-slow-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture roll

preflight-fast-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --gesture height

preflight-fast-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --gesture roll

preflight-height: preflight-slow-height

preflight-roll: preflight-slow-roll

live-slow-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture height --live

live-slow-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture roll --live

live-fast-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --gesture height --live

live-fast-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_SCRIPT) --gesture roll --live

live-height: live-slow-height

live-roll: live-slow-roll

clean:
	$(COMPOSE) down --remove-orphans
