#############################################################################
# Makefile for building: RHOGL
# Generated by qmake (3.0) (Qt 5.5.0)
# Project:  RHOGL.pro
# Template: subdirs
# Command: /home/kevin/Qt/5.5/gcc_64/bin/qmake -spec linux-g++ -o Makefile RHOGL.pro
#############################################################################

MAKEFILE      = Makefile

first: make_first
QMAKE         = /home/kevin/Qt/5.5/gcc_64/bin/qmake
DEL_FILE      = rm -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p
COPY          = cp -f
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
INSTALL_FILE  = install -m 644 -p
INSTALL_PROGRAM = install -m 755 -p
INSTALL_DIR   = cp -f -R
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
TAR           = tar -cf
COMPRESS      = gzip -9f
DISTNAME      = RHOGL1.0.0
DISTDIR = /home/kevin/Documents/RHOGL/.tmp/RHOGL1.0.0
SUBTARGETS    =  \
		sub-Util \
		sub-Harris3D \
		sub-Symmetry-RANSAC


sub-Util-qmake_all:  FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile
	cd Util/ && $(MAKE) -f Makefile qmake_all
sub-Util: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile
sub-Util-make_first: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile 
sub-Util-all: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile all
sub-Util-clean: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile clean
sub-Util-distclean: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile distclean
sub-Util-install_subtargets: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile install
sub-Util-uninstall_subtargets: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile uninstall
sub-Harris3D-qmake_all: sub-Util-qmake_all FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile
	cd Harris3D/ && $(MAKE) -f Makefile qmake_all
sub-Harris3D: sub-Util FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile
sub-Harris3D-make_first: sub-Util-make_first FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile 
sub-Harris3D-all: sub-Util-all FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile all
sub-Harris3D-clean: sub-Util-clean FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile clean
sub-Harris3D-distclean: sub-Util-distclean FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile distclean
sub-Harris3D-install_subtargets: sub-Util-install_subtargets FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile install
sub-Harris3D-uninstall_subtargets: sub-Util-uninstall_subtargets FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile uninstall
sub-Symmetry-RANSAC-qmake_all: sub-Util-qmake_all sub-Harris3D-qmake_all FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile
	cd Symmetry-RANSAC/ && $(MAKE) -f Makefile qmake_all
sub-Symmetry-RANSAC: sub-Util \
		sub-Harris3D FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile
sub-Symmetry-RANSAC-make_first: sub-Util-make_first sub-Harris3D-make_first FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile 
sub-Symmetry-RANSAC-all: sub-Util-all sub-Harris3D-all FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile all
sub-Symmetry-RANSAC-clean: sub-Util-clean sub-Harris3D-clean FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile clean
sub-Symmetry-RANSAC-distclean: sub-Util-distclean sub-Harris3D-distclean FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile distclean
sub-Symmetry-RANSAC-install_subtargets: sub-Util-install_subtargets sub-Harris3D-install_subtargets FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile install
sub-Symmetry-RANSAC-uninstall_subtargets: sub-Util-uninstall_subtargets sub-Harris3D-uninstall_subtargets FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile uninstall

Makefile: RHOGL.pro ../../Qt/5.5/gcc_64/mkspecs/linux-g++/qmake.conf ../../Qt/5.5/gcc_64/mkspecs/features/spec_pre.prf \
		../../Qt/5.5/gcc_64/mkspecs/common/unix.conf \
		../../Qt/5.5/gcc_64/mkspecs/common/linux.conf \
		../../Qt/5.5/gcc_64/mkspecs/common/sanitize.conf \
		../../Qt/5.5/gcc_64/mkspecs/common/gcc-base.conf \
		../../Qt/5.5/gcc_64/mkspecs/common/gcc-base-unix.conf \
		../../Qt/5.5/gcc_64/mkspecs/common/g++-base.conf \
		../../Qt/5.5/gcc_64/mkspecs/common/g++-unix.conf \
		../../Qt/5.5/gcc_64/mkspecs/qconfig.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dcore.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dcore_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dinput.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dinput_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquick.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquick_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquickrenderer.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquickrenderer_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3drenderer.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3drenderer_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bluetooth.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bluetooth_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bootstrap_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_clucene_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_concurrent.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_concurrent_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_core.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_core_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_dbus.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_dbus_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_declarative.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_declarative_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designer.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designer_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designercomponents_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_enginio.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_enginio_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_gui.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_gui_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_help.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_help_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_location.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_location_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimedia.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimedia_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimediawidgets.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimediawidgets_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_network.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_network_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_nfc.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_nfc_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_opengl.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_opengl_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_openglextensions.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_openglextensions_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_platformsupport_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_positioning.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_positioning_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_printsupport.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_printsupport_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qml.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qml_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmldevtools_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmltest.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmltest_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qtmultimediaquicktools_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quick.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quick_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickparticles_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickwidgets.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickwidgets_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_script.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_script_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_scripttools.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_scripttools_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sensors.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sensors_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_serialport.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_serialport_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sql.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sql_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_svg.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_svg_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_testlib.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_testlib_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uiplugin.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uitools.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uitools_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webchannel.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webchannel_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webengine.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webengine_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginecore.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginecore_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginewidgets.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginewidgets_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkit.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkit_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkitwidgets.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkitwidgets_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_websockets.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_websockets_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webview_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_widgets.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_widgets_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_x11extras.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_x11extras_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xml.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xml_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xmlpatterns.pri \
		../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xmlpatterns_private.pri \
		../../Qt/5.5/gcc_64/mkspecs/features/qt_functions.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/qt_config.prf \
		../../Qt/5.5/gcc_64/mkspecs/linux-g++/qmake.conf \
		../../Qt/5.5/gcc_64/mkspecs/features/spec_post.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/exclusive_builds.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/default_pre.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/resolve_config.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/default_post.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/warn_on.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/testcase_targets.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/exceptions.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/yacc.prf \
		../../Qt/5.5/gcc_64/mkspecs/features/lex.prf \
		RHOGL.pro
	$(QMAKE) -spec linux-g++ -o Makefile RHOGL.pro
../../Qt/5.5/gcc_64/mkspecs/features/spec_pre.prf:
../../Qt/5.5/gcc_64/mkspecs/common/unix.conf:
../../Qt/5.5/gcc_64/mkspecs/common/linux.conf:
../../Qt/5.5/gcc_64/mkspecs/common/sanitize.conf:
../../Qt/5.5/gcc_64/mkspecs/common/gcc-base.conf:
../../Qt/5.5/gcc_64/mkspecs/common/gcc-base-unix.conf:
../../Qt/5.5/gcc_64/mkspecs/common/g++-base.conf:
../../Qt/5.5/gcc_64/mkspecs/common/g++-unix.conf:
../../Qt/5.5/gcc_64/mkspecs/qconfig.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dcore.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dcore_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dinput.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dinput_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquick.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquick_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquickrenderer.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquickrenderer_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3drenderer.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3drenderer_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bluetooth.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bluetooth_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bootstrap_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_clucene_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_concurrent.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_concurrent_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_core.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_core_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_dbus.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_dbus_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_declarative.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_declarative_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designer.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designer_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designercomponents_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_enginio.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_enginio_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_gui.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_gui_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_help.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_help_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_location.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_location_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimedia.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimedia_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimediawidgets.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimediawidgets_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_network.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_network_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_nfc.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_nfc_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_opengl.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_opengl_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_openglextensions.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_openglextensions_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_platformsupport_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_positioning.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_positioning_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_printsupport.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_printsupport_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qml.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qml_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmldevtools_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmltest.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmltest_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qtmultimediaquicktools_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quick.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quick_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickparticles_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickwidgets.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickwidgets_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_script.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_script_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_scripttools.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_scripttools_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sensors.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sensors_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_serialport.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_serialport_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sql.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sql_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_svg.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_svg_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_testlib.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_testlib_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uiplugin.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uitools.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uitools_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webchannel.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webchannel_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webengine.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webengine_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginecore.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginecore_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginewidgets.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginewidgets_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkit.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkit_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkitwidgets.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkitwidgets_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_websockets.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_websockets_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webview_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_widgets.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_widgets_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_x11extras.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_x11extras_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xml.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xml_private.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xmlpatterns.pri:
../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xmlpatterns_private.pri:
../../Qt/5.5/gcc_64/mkspecs/features/qt_functions.prf:
../../Qt/5.5/gcc_64/mkspecs/features/qt_config.prf:
../../Qt/5.5/gcc_64/mkspecs/linux-g++/qmake.conf:
../../Qt/5.5/gcc_64/mkspecs/features/spec_post.prf:
../../Qt/5.5/gcc_64/mkspecs/features/exclusive_builds.prf:
../../Qt/5.5/gcc_64/mkspecs/features/default_pre.prf:
../../Qt/5.5/gcc_64/mkspecs/features/resolve_config.prf:
../../Qt/5.5/gcc_64/mkspecs/features/default_post.prf:
../../Qt/5.5/gcc_64/mkspecs/features/warn_on.prf:
../../Qt/5.5/gcc_64/mkspecs/features/testcase_targets.prf:
../../Qt/5.5/gcc_64/mkspecs/features/exceptions.prf:
../../Qt/5.5/gcc_64/mkspecs/features/yacc.prf:
../../Qt/5.5/gcc_64/mkspecs/features/lex.prf:
RHOGL.pro:
qmake: FORCE
	@$(QMAKE) -spec linux-g++ -o Makefile RHOGL.pro

qmake_all: sub-Util-qmake_all sub-Harris3D-qmake_all sub-Symmetry-RANSAC-qmake_all FORCE

make_first: sub-Util-make_first sub-Harris3D-make_first sub-Symmetry-RANSAC-make_first  FORCE
all: sub-Util-all sub-Harris3D-all sub-Symmetry-RANSAC-all  FORCE
clean: sub-Util-clean sub-Harris3D-clean sub-Symmetry-RANSAC-clean  FORCE
distclean: sub-Util-distclean sub-Harris3D-distclean sub-Symmetry-RANSAC-distclean  FORCE
	-$(DEL_FILE) Makefile
install_subtargets: sub-Util-install_subtargets sub-Harris3D-install_subtargets sub-Symmetry-RANSAC-install_subtargets FORCE
uninstall_subtargets: sub-Util-uninstall_subtargets sub-Harris3D-uninstall_subtargets sub-Symmetry-RANSAC-uninstall_subtargets FORCE

sub-Util-check:
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile check
sub-Harris3D-check: sub-Util-check
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile check
sub-Symmetry-RANSAC-check: sub-Util-check sub-Harris3D-check
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -f Makefile check
check: sub-Util-check sub-Harris3D-check sub-Symmetry-RANSAC-check
install:install_subtargets  FORCE

uninstall: uninstall_subtargets FORCE

FORCE:

dist: distdir FORCE
	(cd `dirname $(DISTDIR)` && $(TAR) $(DISTNAME).tar $(DISTNAME) && $(COMPRESS) $(DISTNAME).tar) && $(MOVE) `dirname $(DISTDIR)`/$(DISTNAME).tar.gz . && $(DEL_FILE) -r $(DISTDIR)

distdir: sub-Util-distdir sub-Harris3D-distdir sub-Symmetry-RANSAC-distdir FORCE
	@test -d $(DISTDIR) || mkdir -p $(DISTDIR)
	$(COPY_FILE) --parents ../../Qt/5.5/gcc_64/mkspecs/features/spec_pre.prf ../../Qt/5.5/gcc_64/mkspecs/common/unix.conf ../../Qt/5.5/gcc_64/mkspecs/common/linux.conf ../../Qt/5.5/gcc_64/mkspecs/common/sanitize.conf ../../Qt/5.5/gcc_64/mkspecs/common/gcc-base.conf ../../Qt/5.5/gcc_64/mkspecs/common/gcc-base-unix.conf ../../Qt/5.5/gcc_64/mkspecs/common/g++-base.conf ../../Qt/5.5/gcc_64/mkspecs/common/g++-unix.conf ../../Qt/5.5/gcc_64/mkspecs/qconfig.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dcore.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dcore_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dinput.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dinput_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquick.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquick_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquickrenderer.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3dquickrenderer_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3drenderer.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_3drenderer_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bluetooth.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bluetooth_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_bootstrap_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_clucene_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_concurrent.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_concurrent_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_core.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_core_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_dbus.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_dbus_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_declarative.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_declarative_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designer.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designer_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_designercomponents_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_enginio.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_enginio_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_gui.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_gui_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_help.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_help_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_location.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_location_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimedia.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimedia_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimediawidgets.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_multimediawidgets_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_network.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_network_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_nfc.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_nfc_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_opengl.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_opengl_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_openglextensions.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_openglextensions_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_platformsupport_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_positioning.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_positioning_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_printsupport.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_printsupport_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qml.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qml_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmldevtools_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmltest.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qmltest_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_qtmultimediaquicktools_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quick.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quick_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickparticles_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickwidgets.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_quickwidgets_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_script.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_script_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_scripttools.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_scripttools_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sensors.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sensors_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_serialport.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_serialport_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sql.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_sql_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_svg.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_svg_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_testlib.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_testlib_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uiplugin.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uitools.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_uitools_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webchannel.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webchannel_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webengine.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webengine_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginecore.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginecore_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginewidgets.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webenginewidgets_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkit.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkit_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkitwidgets.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webkitwidgets_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_websockets.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_websockets_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_webview_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_widgets.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_widgets_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_x11extras.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_x11extras_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xml.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xml_private.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xmlpatterns.pri ../../Qt/5.5/gcc_64/mkspecs/modules/qt_lib_xmlpatterns_private.pri ../../Qt/5.5/gcc_64/mkspecs/features/qt_functions.prf ../../Qt/5.5/gcc_64/mkspecs/features/qt_config.prf ../../Qt/5.5/gcc_64/mkspecs/linux-g++/qmake.conf ../../Qt/5.5/gcc_64/mkspecs/features/spec_post.prf ../../Qt/5.5/gcc_64/mkspecs/features/exclusive_builds.prf ../../Qt/5.5/gcc_64/mkspecs/features/default_pre.prf ../../Qt/5.5/gcc_64/mkspecs/features/resolve_config.prf ../../Qt/5.5/gcc_64/mkspecs/features/default_post.prf ../../Qt/5.5/gcc_64/mkspecs/features/warn_on.prf ../../Qt/5.5/gcc_64/mkspecs/features/testcase_targets.prf ../../Qt/5.5/gcc_64/mkspecs/features/exceptions.prf ../../Qt/5.5/gcc_64/mkspecs/features/yacc.prf ../../Qt/5.5/gcc_64/mkspecs/features/lex.prf RHOGL.pro $(DISTDIR)/

sub-Util-distdir: FORCE
	@test -d Util/ || mkdir -p Util/
	cd Util/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Util/Util.pro -spec linux-g++ -o Makefile ) && $(MAKE) -e -f Makefile distdir DISTDIR=$(DISTDIR)/Util

sub-Harris3D-distdir: FORCE
	@test -d Harris3D/ || mkdir -p Harris3D/
	cd Harris3D/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Harris3D/Harris3D.pro -spec linux-g++ -o Makefile ) && $(MAKE) -e -f Makefile distdir DISTDIR=$(DISTDIR)/Harris3D

sub-Symmetry-RANSAC-distdir: FORCE
	@test -d Symmetry-RANSAC/ || mkdir -p Symmetry-RANSAC/
	cd Symmetry-RANSAC/ && ( test -e Makefile || $(QMAKE) /home/kevin/Documents/RHOGL/Symmetry-RANSAC/Symmetry-RANSAC.pro -spec linux-g++ -o Makefile ) && $(MAKE) -e -f Makefile distdir DISTDIR=$(DISTDIR)/Symmetry-RANSAC
