import tesseract
import os
import re
import numpy as np


def test_environment():

    TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    with open(os.path.join(TESSERACT_SUPPORT_DIR, "urdf", "abb_irb2400.urdf"), "r") as f:
        abb_irb2400_urdf = f.read()

    with open(os.path.join(TESSERACT_SUPPORT_DIR, "urdf", "abb_irb2400.srdf"), "r") as f:
        abb_irb2400_srdf = f.read()

    t = tesseract.Tesseract()

    t.init(abb_irb2400_urdf, abb_irb2400_srdf, TesseractSupportResourceLocator())

    t_srdf = t.getSRDFModel()

    print("robot name: " + str(t_srdf.getName()))
    assert str(t_srdf.getName()) == "abb_irb2400"

    chain_groups = t_srdf.getChainGroups()
    assert len(chain_groups) == 1
    group = chain_groups["manipulator"]
    assert(len(group) == 1)
    assert group[0] == ("base_link", "tool0")

    # Group state now
    group_states = t_srdf.getGroupStates()
    assert len(group_states) == 1
    group_state = group_states["manipulator"]
    state = group_state["all-zeros"]
    for i in range(1, 7):        
        assert state["joint_{}".format(i)] == 0.0


class TesseractSupportResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(TesseractSupportResourceLocator, self).__init__()
        self.TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    def locateResource(self, url):

        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$", url)
        if url_match is None:
            return None

        fname = os.path.join(self.TESSERACT_SUPPORT_DIR, os.path.normpath(url_match.group(1)))
        with open(fname, "rb") as f:
            resource_bytes = f.read()

        resource = tesseract.BytesResource(url, resource_bytes)

        return resource


if __name__ == "__main__":
    test_environment()
