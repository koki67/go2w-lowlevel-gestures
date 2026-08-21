COMPOSE := docker compose
SERVICE := controller
PYTHON := /opt/venv/bin/python
SLOW_SCRIPT := /app/go2w_gesture_real.py
FAST_SCRIPT := /app/go2w_gesture_real_fast.py
NO_TRACKING_STOP_SCRIPT := /app/go2w_gesture_real_no_tracking_stop.py
FAST_NO_TRACKING_STOP_SCRIPT := /app/go2w_gesture_real_fast_no_tracking_stop.py
ADAPTIVE_SCRIPT := /app/go2w_gesture_real_adaptive.py
HOST_PYTHON ?= python3
UNITREE_MUJOCO_ROOT ?= $(abspath ../unitree_mujoco)
UNITREE_MUJOCO_PYTHON ?= $(UNITREE_MUJOCO_ROOT)/simulate_python/.venv/bin/python
SIM_ENV := UNITREE_MUJOCO_ROOT='$(UNITREE_MUJOCO_ROOT)' \
	UNITREE_MUJOCO_PYTHON='$(UNITREE_MUJOCO_PYTHON)'
SIM_DIR := simulation
SIM_ARGS ?=
SIM_RUN_ARGS := $(SIM_ARGS)
ifneq ($(filter save-plot,$(MAKECMDGOALS)),)
SIM_RUN_ARGS += --save-plot
endif

.PHONY: help build test clean describe \
	describe-slow describe-slow-height describe-slow-roll \
	describe-fast describe-fast-height describe-fast-roll \
	describe-no-tracking-stop describe-no-tracking-stop-height describe-no-tracking-stop-roll \
	describe-fast-no-tracking-stop describe-fast-no-tracking-stop-height describe-fast-no-tracking-stop-roll \
	describe-adaptive-height describe-adaptive-roll \
	describe-height describe-roll \
	preflight-slow-height preflight-slow-roll \
	preflight-fast-height preflight-fast-roll \
	preflight-no-tracking-stop-height preflight-no-tracking-stop-roll \
	preflight-fast-no-tracking-stop-height preflight-fast-no-tracking-stop-roll \
	preflight-adaptive-height preflight-adaptive-roll \
	preflight-height preflight-roll \
	live-slow-height live-slow-roll \
	live-fast-height live-fast-roll \
	live-no-tracking-stop-height live-no-tracking-stop-roll \
	live-fast-no-tracking-stop-height live-fast-no-tracking-stop-roll \
	live-adaptive-height live-adaptive-roll \
	live-height live-roll \
	sim-doctor sim-describe sim-describe-height sim-describe-roll \
	sim-describe-quick-stand sim-describe-shake-off \
	sim-height sim-roll sim-quick-stand sim-shake-off save-plot

help:
	@echo "Non-hardware: make build | test | describe"
	@echo "Slow 2.0/2.0 read-only: make preflight-slow-height | preflight-slow-roll"
	@echo "Slow 2.0/2.0 physical:  make live-slow-height | live-slow-roll"
	@echo "Fast 1.0/0.5 read-only: make preflight-fast-height | preflight-fast-roll"
	@echo "Fast 1.0/0.5 physical:  make live-fast-height | live-fast-roll"
	@echo "Slow 2.0/2.0, no tracking-error stop: make preflight-no-tracking-stop-height | preflight-no-tracking-stop-roll"
	@echo "Slow 2.0/2.0, no tracking-error stop: make live-no-tracking-stop-height | live-no-tracking-stop-roll"
	@echo "Fast 1.0/0.5, no tracking-error stop: make preflight-fast-no-tracking-stop-height | preflight-fast-no-tracking-stop-roll"
	@echo "Fast 1.0/0.5, no tracking-error stop: make live-fast-no-tracking-stop-height | live-fast-no-tracking-stop-roll"
	@echo "Adaptive fast read-only: make preflight-adaptive-height | preflight-adaptive-roll"
	@echo "Adaptive fast physical:  make live-adaptive-height | live-adaptive-roll"
	@echo "Compatibility: preflight-height/roll and live-height/roll use slow"
	@echo "MuJoCo requirement check: make sim-doctor"
	@echo "MuJoCo runs: make sim-height | sim-roll | sim-quick-stand | sim-shake-off"
	@echo "Quick stand: low -> high in 0.1 s (simulation only)"
	@echo "Shake off: 8 rapid right/left cycles (simulation only)"
	@echo "Save a MuJoCo joint plot: make sim-height save-plot"

build:
	$(COMPOSE) build --pull

describe: describe-slow describe-fast describe-no-tracking-stop describe-fast-no-tracking-stop

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

describe-no-tracking-stop:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --describe

describe-no-tracking-stop-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --gesture height --describe

describe-no-tracking-stop-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --gesture roll --describe

describe-fast-no-tracking-stop:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --describe

describe-fast-no-tracking-stop-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --gesture height --describe

describe-fast-no-tracking-stop-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --gesture roll --describe

describe-adaptive-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(ADAPTIVE_SCRIPT) --gesture height --describe

describe-adaptive-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(ADAPTIVE_SCRIPT) --gesture roll --describe

describe-height: describe-slow-height

describe-roll: describe-slow-roll

test:
	$(COMPOSE) run --rm --no-deps -T \
		--entrypoint /opt/venv/bin/python $(SERVICE) \
		-m unittest discover -s /app/tests -v

sim-doctor:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_height_sequence_sim.py --doctor

sim-describe: sim-describe-height sim-describe-roll sim-describe-quick-stand \
	sim-describe-shake-off

sim-describe-height:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_height_sequence_sim.py --describe

sim-describe-roll:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_roll_sequence_sim.py --describe

sim-describe-quick-stand:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_quick_stand_sequence_sim.py --describe

sim-describe-shake-off:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_shake_off_sequence_sim.py --describe

sim-height:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_height_sequence_sim.py $(SIM_RUN_ARGS)

sim-roll:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_roll_sequence_sim.py $(SIM_RUN_ARGS)

sim-quick-stand:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_quick_stand_sequence_sim.py $(SIM_RUN_ARGS)

sim-shake-off:
	$(SIM_ENV) $(HOST_PYTHON) $(SIM_DIR)/go2w_shake_off_sequence_sim.py $(SIM_RUN_ARGS)

save-plot:
	@:

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

preflight-no-tracking-stop-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --gesture height

preflight-no-tracking-stop-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --gesture roll

preflight-fast-no-tracking-stop-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --gesture height

preflight-fast-no-tracking-stop-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --gesture roll

preflight-adaptive-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(ADAPTIVE_SCRIPT) --gesture height

preflight-adaptive-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(ADAPTIVE_SCRIPT) --gesture roll

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

live-no-tracking-stop-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --gesture height --live

live-no-tracking-stop-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(NO_TRACKING_STOP_SCRIPT) --gesture roll --live

live-fast-no-tracking-stop-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --gesture height --live

live-fast-no-tracking-stop-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(FAST_NO_TRACKING_STOP_SCRIPT) --gesture roll --live

live-adaptive-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(ADAPTIVE_SCRIPT) --gesture height --live

live-adaptive-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(ADAPTIVE_SCRIPT) --gesture roll --live

live-height: live-slow-height

live-roll: live-slow-roll

clean:
	$(COMPOSE) down --remove-orphans
