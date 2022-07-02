cd build
sudo rm -rf bin/resources/html
sudo find /usr -name open3d -exec rm -rf {} \;
make && sudo make install-pip-package && cd ..