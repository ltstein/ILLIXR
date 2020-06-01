#LDFLAGS = $(shell pkg-config opencv --libs)
#CFLAGS = $(shell pkg-config opencv --cflags)
#include common/common.mk

.PHONY: plugin.dbg.so
plugin.dbg.so:
	mkdir -p zed-opencv/cpp/build/ && \
	cd zed-opencv/cpp/build/ && \
	cmake .. && \
	make "-j$(nproc)" && \
	cd ../../.. && \
	rm -f plugin.dbg.so && \
	ln -s zed-opencv/cpp/build/ZED_with_OpenCV.so plugin.dbg.so && \
	true

.PHONY: plugin.opt.so
plugin.opt.so:
	$(error "This plugin does not yet support an optimized build")

.PHONY: clean
clean:
	touch zed-opencv/cpp/build && rm -rf zed-opencv/cpp/build *.so

