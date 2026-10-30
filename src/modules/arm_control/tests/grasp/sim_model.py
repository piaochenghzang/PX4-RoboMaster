"""Shared experiment-copy model edits; no transport, processes or actuation."""
import xml.etree.ElementTree as ET


def set_pad_patch(collision,radius_mm):
    """Finite, force-dependent torsional pad friction, not an attachment.

    ODE combines patch radii by MAX and torsional coefficients by MIN.
    Only the two finger surfaces are changed; cube/support torsion stays
    at its zero-radius default. 4 mm is a demo pad model, not a calibration.
    """
    friction=collision.find('surface/friction')
    if friction is None: raise RuntimeError('pad friction element missing')
    torsion=friction.find('torsional')
    if torsion is None: torsion=ET.SubElement(friction,'torsional')
    for name,value in [('coefficient','.8'),('use_patch_radius','true'),('patch_radius',str(radius_mm*.001))]:
        element=torsion.find(name)
        if element is None: element=ET.SubElement(torsion,name)
        element.text=value


def remove_lidar(model):
    """Remove only the expanded lidar links/mounts in this experiment copy."""
    lidar=[link for link in model.findall('link') if link.get('name','').startswith('velodyne::')]
    if not lidar: raise RuntimeError('cannot identify expanded velodyne links for no-lidar scene')
    names={link.get('name') for link in lidar}
    for joint in list(model.findall('joint')):
        if joint.findtext('parent') in names or joint.findtext('child') in names:
            model.remove(joint)
    for link in lidar: model.remove(link)
