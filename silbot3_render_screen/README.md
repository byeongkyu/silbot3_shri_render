# silbot3_render_screen

## prerequisites

This package uses QtWebEngine instead of QtWebKit. So you shoud install Qt5.9, PyQt5 5.9, sip.

* Qt 5.9.1

        $ sudo add-apt-repository ppa:beineri/opt-qt591-xenial
        $ sudo apt-get update
        $ sudo apt-get install qt59-meta-full

* sip

        $ wget https://sourceforge.net/projects/pyqt/files/sip/sip-4.19.3/sip-4.19.3.tar.gz
        $ tar zxf sip-4.19.3.tar.gz
        $ cd sip-4.19.3
        $ python configure.py
        $ make
        $ sudo make install

* PyQt5

        $ wget https://sourceforge.net/projects/pyqt/files/PyQt5/PyQt-5.9/PyQt5_gpl-5.9.tar.gz
        $ tar zxf PyQt5_gpl-5.9.tar.gz
        $ cd PyQt5_gpl-5.9
        $ python configure.py
        $ make
        $ sudo make install

* Remove old library

        $ cd /usr/lib/python2.7/dist-packages
        $ mv sip.x86_64-linux-gnu.so sip.x86_64-linux-gnu.so.backup
        $ cd /usr/lib/python2.7/dist-packages/PyQt5
        $ sudo mkdir backup
        $ sudo mv *.x86_64-linux-gnu.so ./backup

## usage
        $ rosrun silbot3_render_screen rneder_screen.py _resouce_path:=<your resource path> _use_full_screen:=<true|false>
