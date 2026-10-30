"""Batched equivalent of the existing three-frame box distance lower bound."""
import numpy as np


def _pack(boxes):
    boxes=list(boxes)
    if not boxes or any(len(box)!=3 for box in boxes):raise ValueError('BOX_COLLECTION')
    poses=np.asarray([box[1] for box in boxes],dtype=float)
    half=np.asarray([box[2] for box in boxes],dtype=float)
    if (poses.shape!=(len(boxes),4,4) or half.shape!=(len(boxes),3) or
            not np.isfinite(poses).all() or not np.isfinite(half).all() or np.any(half<=0)):
        raise ValueError('BOX_SHAPE_OR_NONFINITE')
    rotation=poses[:,:3,:3]
    if (not np.allclose(poses[:,3,:],[0.,0.,0.,1.],atol=1e-9,rtol=0) or
            not np.allclose(np.swapaxes(rotation,1,2)@rotation,np.eye(3),atol=1e-8,rtol=1e-6) or
            not np.allclose(np.linalg.det(rotation),1.,atol=1e-8,rtol=1e-6)):
        raise ValueError('BOX_NOT_RIGID')
    return rotation,poses[:,:3,3],half


def box_gaps(first,second):
    """Return all N-by-M conservative gaps, exactly using world/A/B axes.

    This is the same lower bound as perception.check_installation.box_gap,
    not the exact Euclidean OBB distance. Zero means separation is unproven,
    not a measured penetration depth. Inputs are validated once per batch.
    """
    ar,at,ah=_pack(first);br,bt,bh=_pack(second)
    delta=bt[None,:,:]-at[:,None,:]
    ae=np.einsum('nij,nj->ni',np.abs(ar),ah)
    be=np.einsum('mij,mj->mi',np.abs(br),bh)
    world=np.linalg.norm(np.maximum(np.abs(delta)-ae[:,None,:]-be[None,:,:],0.),axis=2)
    relative=np.einsum('nki,mkj->nmij',ar,br)
    in_a=np.einsum('nki,nmk->nmi',ar,delta)
    in_b=np.einsum('mki,nmk->nmi',br,delta)
    be_a=np.einsum('nmij,mj->nmi',np.abs(relative),bh)
    ae_b=np.einsum('nmji,nj->nmi',np.abs(relative),ah)
    local_a=np.linalg.norm(np.maximum(np.abs(in_a)-ah[:,None,:]-be_a,0.),axis=2)
    local_b=np.linalg.norm(np.maximum(np.abs(in_b)-bh[None,:,:]-ae_b,0.),axis=2)
    result=np.maximum(np.maximum(world,local_a),local_b)
    if not np.isfinite(result).all():raise ValueError('BOX_GAP_OVERFLOW')
    return result
