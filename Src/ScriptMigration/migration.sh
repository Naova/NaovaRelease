OSVersion=`lsb_release -r | cut -d$'\t' -f2 | cut -d"." -f1`
sudo apt remove clang-4.0 clang
if [ $OSVersion = "18" ]
then
    sudo apt-get install -y clang-10

    sudo update-alternatives --remove-all clang
    sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-10 100
    sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
else
    
    sudo apt-get install -y clang
fi

echo "Installing clang..."
clangVersion=""
if [ $OSVersion = "22" ]
then
    clangVersion=`clang --version | head -n1 | cut -d" " -f4 | cut -d"-" -f1`
else
    clangVersion=`clang --version | head -n1 | cut -d" " -f3 | cut -d"-" -f1`
fi

rm -rf ../../Util/Buildchain/clang/

clangDirectory="clang-${clangVersion}.src"
wget "https://github.com/llvm/llvm-project/releases/download/llvmorg-${clangVersion}/${clangDirectory}.tar.xz"
echo "Decompressing ${clangDirectory}.tar.xz..."
tar -xf "${clangDirectory}.tar.xz"
mkdir -p ../../Util/Buildchain/clang/include
cp $clangDirectory/lib/Headers/* ../../Util/Buildchain/clang/include/
rm -rf $clangDirectory*
sed -i '25s/;/ throw();/' ../../Util/Buildchain/clang/include/mm_malloc.h
sed -i '/t;/ t=0;/' ../../Util/Buildchain/clang/include/cetintrin.h