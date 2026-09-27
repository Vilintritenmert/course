compile: 
	cmake --preset debug && cmake --build --preset debug

test:
	ctest --test-dir build/debug/homework_06 --output-on-failure

clean:
	rm -rf build

format:
	@if git clang-format --help >/dev/null 2>&1; then \
		git clang-format --diff --staged -q; \
	elif command -v git-clang-format >/dev/null 2>&1 && [ -x "$$(command -v git-clang-format)" ]; then \
		git-clang-format --diff --staged -q; \
	else \
		echo "error: git-clang-format is not installed or is not executable."; \
		echo "Install clang-format tooling and retry, for example: sudo apt install clang-format"; \
		exit 127; \
	fi

quality:
	run-clang-tidy -p build/debug ${FOLDER}

quality-file:
	clang-tidy -p build/debug ${FILE}
