#!/usr/bin/env bash

# URL to the latest FreeCAD AppImage release, updated manually
FREECAD_RELEASE_URL="https://github.com/FreeCAD/FreeCAD/releases/download/1.0.2/FreeCAD_1.0.2-conda-Linux-x86_64-py311.AppImage"

# Validate target argument to be install or remove
if [ "$1" != "install" ] && [ "$1" != "remove" ]; then
    echo "Usage: $0 [install|remove]"
    exit 1
fi

# If install, perform the following steps:
# - Create /usr/local/bin/appimages directory if it doesn't exist
# - Download the latest AppImages from FREECAD_RELEASE_URL
# - Copy the FreeCAD AppImage to /usr/local/bin/appimages/
# - Make the FreeCAD AppImage executable
# - Create a symbolic link /usr/local/bin/freecad pointing to the FreeCAD AppImage
if [ "$1" == "install" ]; then
    echo "Installing FreeCAD AppImage..."
    sudo mkdir -p /usr/local/bin/appimages/
    # Check if APPIMAGE already exists in /usr/local/bin/appimages/
    APPIMAGE_NAME=$(basename $FREECAD_RELEASE_URL)
    if [ -f /usr/local/bin/appimages/$APPIMAGE_NAME ]; then
        echo "FreeCAD AppImage already exists. Keeping the existing file."
    else
        echo "Downloading FreeCAD AppImage..."
        wget -q -P /tmp $FREECAD_RELEASE_URL
    fi
    echo "Copying FreeCAD AppImage to /usr/local/bin/appimages/ ..."
    sudo cp /tmp/$APPIMAGE_NAME /usr/local/bin/appimages/
    sudo chmod +x /usr/local/bin/appimages/$APPIMAGE_NAME
    echo "Creating symbolic link /usr/local/bin/freecad ..."
    sudo ln -sf /usr/local/bin/appimages/$APPIMAGE_NAME /usr/local/bin/freecad
    echo "FreeCAD AppImage installed successfully."
fi

# If remove, perform the following steps:
# - Remove the symbolic link /usr/local/bin/freecad
# - Remove the FreeCAD AppImage from /usr/local/bin/appimages/
if [ "$1" == "remove" ]; then
    echo "Removing symbolci link /usr/local/bin/freecad ..."
    sudo rm -f /usr/local/bin/freecad
    # Check if the AppImage exists in /usr/local/bin/appimages/
    APPIMAGE_NAME=$(basename $FREECAD_RELEASE_URL)
    if [ -f /usr/local/bin/appimages/$APPIMAGE_NAME ]; then
        echo "Removing FreeCAD AppImage..."
        sudo rm -f /usr/local/bin/appimages/$APPIMAGE_NAME
    else
        echo "FreeCAD AppImage not found in /usr/local/bin/appimages/."
        echo "Remove the AppImage manually if needed!"
    fi
    echo "FreeCAD AppImage removed successfully."
fi
