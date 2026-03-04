git lfs install
git lfs track "*.rdk" "*.sld" "*.robot"
git add .gitattributes
git commit -m "Track RoboDK assets with Git LFS"

mkdir -p third_party

git submodule add https://gitlab.com/libeigen/eigen.git third_party/eigen
git -C third_party/eigen fetch --tags --prune
git -C third_party/eigen checkout 5.0.1

# record the submodule + commit the pin
git add .gitmodules third_party/eigen
git commit -m "Vendor Eigen 5.0.1 as submodule in third_party/eigen"