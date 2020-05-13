#!/bin/bash

echo "================================================================"
echo "Installing Scilab"
echo "================================================================"

scilab_version="6.1.0"
scilab_download_link="https://www.scilab.org/download/6.1.0/scilab-6.1.0.bin.linux-x86_64.tar.gz"
scilab_executable_path="/usr/local/bin/scilab"


# Check if scilab is already installed
if [ -e "/opt/scilab-${scilab_version}/bin/scilab" ] && [ -e ${scilab_executable_path}-${scilab_version} ]
then
        echo "================================================================"
        echo "Scilab is already installed"
        echo "================================================================"
else
    # Download
    #wget -O /tmp/scilab-${scilab_version}.tar.gz "https://www.scilab.org/download/6.1.0/scilab-6.1.0.bin.linux-x86_64.tar.gz"

    # Unzip and symlink to usr location
    sudo mkdir -p /opt/scilab-${scilab_version}
    sudo tar xfz /tmp/scilab-${scilab_version}.tar.gz -C /opt/scilab-${scilab_version} --strip-components=1
    
    # Install desktop entry
    cp /opt/scilab-${scilab_version}/share/applications/scilab.desktop ~/.local/share/applications/scilab.desktop 
    echo "Icon=/opt/scilab-${scilab_version}/share/icons/hicolor/256x256/apps/scilab.png" >> ~/.local/share/applications/scilab.desktop
fi

echo "================================================================"
echo "Symlinking Scilab"
echo "================================================================"
# Symlink binary to a location in the PATH
# We do this outside the above `if` because this gives us the ability to 
# change versions without re-downloading
sudo ln -s -f /opt/scilab-${scilab_version}/bin/scilab ${scilab_executable_path}-${scilab_version} sudo ln -s -f ${scilab_executable_path}-${scilab_version} $scilab_executable_path 
echo "================================================================"
echo "Done"
echo "================================================================"
