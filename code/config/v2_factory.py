"""Future v2 component builders, kept explicit until components exist."""


def build_differential_drive(*args, **kwargs):
    raise NotImplementedError("DifferentialDrive is introduced in migration step 07")


def build_t265_source(*args, **kwargs):
    raise NotImplementedError("T265 source is introduced in migration step 08")


def build_pose_fusion(*args, **kwargs):
    raise NotImplementedError("pose fusion is introduced in migration step 10")
