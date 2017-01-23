TEMPLATE = subdirs

CONFIG += \
        warn_on

SUBDIRS +=  Util \
            Harris3D \
            Symmetry-RANSAC

Harris3D.depends        += Util
Symmetry-RANSAC.depends += Util Harris3D
