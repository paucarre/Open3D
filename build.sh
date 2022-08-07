cd build
sudo rm -rf bin/resources/html
sudo find /usr -name open3d* -exec rm -rf {} \;
make -j 2 && make install-pip-package && cd ..
