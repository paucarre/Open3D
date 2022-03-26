
dependencies=(
    3rdparty
    assimp
    boringssl
    civetweb
    curl
    docs
    eigen
    embree
    examples
    filament
    fmt
    imgui
    ippicv
    jsoncpp
    lib
    libpng
    mkl
    mkl_include
    mkl_install
    msgpack-c
    nanoflann
    parallelstl
    poisson
    qhull
    tbb
    tinygltf
    tinyobjloader
    turbojpeg
    webrtc
    zeromq
    zlib
)

for package in "${dependencies[@]}"; do
    mkdir "$package"/lib/x86_64-linux-gnu
done