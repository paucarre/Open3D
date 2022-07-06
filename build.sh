cd build
sudo rm -rf bin/resources/html
sudo find /usr -name open3d -exec rm -rf {} \;
make && make install-pip-package && cd ..