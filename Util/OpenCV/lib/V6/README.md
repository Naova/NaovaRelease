Commande CMake utilisée pour compiler opencv sur le robot

cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_SHARED_LIBS=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_CXX_STANDARD=17 -DBUILD_opencv_core=ON -DBUILD_opencv_imgproc=ON -DBUILD_opencv_core=ON -DBUILD_opencv_highgui=OFF -DBUILD_opencv_ml=OFF -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_calib3d=OFF -DBUILD_opencv_features2d=OFF -DBUILD_opencv_flann=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_stiching=OFF -DBUILD_opencv_video=OFF -DBUILD_opencv_videoio=OFF -DBUILD_opencv_videostab=OFF -DBUILD_opencv_world=OFF -DBUILD_opencv_java=OFF -DBUILD_opencv_python=OFF -DBUILD_opencv_dnn=OFF -DBUILD_LIST=core,highgui,imgproc -DBUILD_opencv_imgcodecs=OFF -D WITH_IPP=ON -D WITH_TBB=ON -D WITH_ITT=OFF -D CMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=1" -DWITH_TIFF=OFF -march=silvermont -mtune=silvermont ..


