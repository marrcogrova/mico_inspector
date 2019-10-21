
# It may happens that vtk lib link without version is not created but still referenced, so create a soft link 
sudo ln -s /usr/bin/vtk6 /usr/bin/vtk
sudo ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.x86_64-linux-gnu.so /usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so
