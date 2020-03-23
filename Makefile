
# Whether to turn compiler warnings into errors
export WERROR ?= false
export BUILD_DIR ?= cmake-build
MAKEFLAGS += --no-print-directory
default: release

release:
	mkdir -p ./$(BUILD_DIR) && cd ./$(BUILD_DIR) && cmake ../ -DCMAKE_BUILD_TYPE=Release -DWERROR=$(WERROR) && cmake --build . -j$(nproc)

debug:
	mkdir -p ./$(BUILD_DIR) && cd ./$(BUILD_DIR) && cmake ../ -DCMAKE_BUILD_TYPE=Debug -DWERROR=$(WERROR) && cmake --build . -j$(nproc)

test:
	@if [ -f ./$(BUILD_DIR)/unit-tests ]; then ./$(BUILD_DIR)/unit-tests; else echo "Please run 'make release' or 'make debug' first" && exit 1; fi

bench:
	@if [ -f ./$(BUILD_DIR)/bench-tests ]; then ./$(BUILD_DIR)/bench-tests; else echo "Please run 'make release' or 'make debug' first" && exit 1; fi

# tidy:
# 	./scripts/clang-tidy.sh

# coverage:
# 	./scripts/coverage.sh

clean:
	rm -rf ./$(BUILD_DIR)
	# remove remains from running 'make coverage'
	rm -f *.profraw
	rm -f *.profdata
	@echo "run 'make distclean' to also clear mason_packages, .mason, and .toolchain directories"

distclean: clean
	rm -rf mason_packages
	rm -rf $(BUILD_DIR)
	# remove remains from running './scripts/setup.sh'
	rm -rf .mason
	rm -rf .toolchain
	rm -f local.env

# format:
# 	./scripts/format.sh

.PHONY: test bench