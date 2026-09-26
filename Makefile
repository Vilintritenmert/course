compile: 
	cmake --preset debug && cmake --build --preset debug

test:
	ctest --test-dir build/debug/homework_06 --output-on-failure

clean:
	rm -rf build

format:
	git-clang-format --diff --staged -q

quality:
	run-clang-tidy -p build/debug ${FOLDER}

quality-file:
	clang-tidy -p build/debug ${FILE}
