1. Clone from the branch `vsyncTimestampNew` and do the basic setup for ILLIXR.
   ```
   git clone --recursive --branch vsyncTimestampNew https://github.com/ILLIXR/ILLIXR.git
   git submodule update --init --recursive
   ./install_deps.bash
   export CXX=clang++-10
   make run.opt
   ```
2. Add `ILLIXR_DATA` to the environment setup. Also we need the `plugin.opt.so`s and `rumtime/main.opt.exe` for Apitrace
   ```
   export ILLIXR_DATA=/path/to/data1/
   make run.opt
   ```
3. Make sure to run `make run.opt` with both `plugins = pose_lookup/ gldemo/ timewarp_gl/ audio_pipeline/ audio_decoding/` and `plugins = offline_imu_cam/ open_vins/ pose_prediction/ gldemo/ timewarp_gl/ audio_pipeline/ audio_decoding/`
4. Now we are setting up Apitrace. The reference is here: https://github.com/apitrace/apitrace/blob/master/docs/INSTALL.markdown. Download Apitrace.
   ```
   git clone https://github.com/apitrace/apitrace
   ```
5. Go into the apitrace directory and build it.
   ```
   cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=RelWithDebInfo
   make -C build
   ```
6. Create two directories to contain the dumped images. One directory is for the `ideal` setup, the other on is for the `actual` setup. In the `ideal` directory:
   ```
   /path/to/apitrace/build/apitrace trace -o illixr.trace /path/to/ILLIXR/runtime/main.opt.exe /path/to/pose_lookup/plugin.opt.so /path/to/gldemo/plugin.opt.so /path/to/timewarp_gl/plugin.opt.so /path/to/audio_pipeline/plugin.opt.so /path/to/audio_decoding/plugin.opt.so
   /path/to/apitrace/build/apitrace dump-images illixr.trace
   rm -rf metrics/
   rm illixr.trace
   ```
   In the `actual` directory:
   ```
   /path/to/apitrace/build/apitrace trace -o illixr.trace /path/to/ILLIXR/runtime/main.opt.exe /path/to/offline_imu_cam/plugin.opt.so /path/to/open_vins/plugin.opt.so /path/to/pose_prediction/plugin.opt.so /path/to/gldemo/plugin.opt.so /path/to/timewarp_gl/plugin.opt.so /path/to/audio_pipeline/plugin.opt.so /path/to/audio_decoding/plugin.opt.so
   /path/to/apitrace/build/apitrace dump-images illixr.trace
   rm -rf metrics/
   ```
7. Delete the first 300 images in both the `ideal` and the `actual` directories. On Jetson, you can change line 33 in `ssim.py` from `for i in range(0, len(testList)):` to `for i in range(300, len(testList)):`.
8. Before editting `ssim.py`, you may need to install some modules.Use the following command:
   ```
   pip3 install scikit-image opencv-python imutils os
   ```
9. Edit `ssim.py` in the ILLIXR root directory. Change line 18 and 19 (the `groundTruthPath` variable and the `testPath` variable) to the following:
   ```
   groundTruthPath = "/path/to/ideal_directory"
   testPath = "/path/to/actual_directory"
   ```
10.Go to the ILLIXR root directory and run `ssim.py`. You can ignore the warning. It's gonna take a few minutes to get the final results printed on screen.
   ```
   python3 ssim.py
   ```
