# HACK g2oconfig.cmake because it complains about not finding eigen for some reason
sudo rm /usr/local/lib/cmake/g2o/g2oConfig.cmake -f

echo "include(CMakeFindDependencyMacro)

find_dependency(OpenGL)

include("/usr/local/lib/cmake/g2o/g2oTargets.cmake")" | sudo tee -a /usr/local/lib/cmake/g2o/g2oConfig.cmake
