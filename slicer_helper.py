""" slicer_helper
Author: Henry Phalen

BSD 3-Clause License

Copyright (c) 2020, Henry Phalen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 """


import slicer
import time
import numpy as np


def make_igtl_node(ip, port, name):
    """ Creates an IGT_link node in Slicer that can be used to communicate with e.g. ROS
    INPUT: ip   [str]  - IP address, (accepts 'localhost')
           port [int]  - Port number
           name [str]  - Connector name
    OUPUT: igtl_connector [vtkMRMLIGTLConnectorNode] """
    igtl_connector = slicer.vtkMRMLIGTLConnectorNode()
    slicer.mrmlScene.AddNode(igtl_connector)
    igtl_connector.SetName(name)
    igtl_connector.SetTypeClient(ip, port)
    igtl_connector.Start()
    return igtl_connector


class SlicerMeshModel:
    """ Takes a mesh model (stl or dae), imports it into Slicer, and sets up a transform in Slicer that the mesh model
    observes and follows automatically """
    def __init__(self, transform_name, mesh_filename):
        """INPUT: transform_name [str] - name assigned to node in Slicer. (If using IGTL be sure this matches)
                  mesh_filename  [str] - filename with given mesh - accepts .stl and .dae """
        self.transform_name = transform_name
        self.mesh_filename = mesh_filename
        _, self.mesh_model_node = slicer.util.loadModel(mesh_filename, returnNode=True)  # TODO: [May become deprecated]
        self.mesh_nodeID = self.mesh_model_node.GetID()
        self.transform_node = slicer.vtkMRMLTransformNode()
        self.transform_node.SetName(transform_name)
        slicer.mrmlScene.AddNode(self.transform_node)
        self.transform_nodeID = self.transform_node.GetID()
        self.mesh_model_node.SetAndObserveTransformNodeID(self.transform_nodeID)


class SlicerVolumeModel:
    """ Takes a volume (nrrd, perhaps others), imports it into Slicer as segment. Allows for voxel manipulation from np
    array """
    def __init__(self, volume_filename):
        """INPUT: volume_filename  [str] - filename with given mesh - accepts .stl and .dae """
        _, self.label_volumeNode = slicer.util.loadLabelVolume(volume_filename, returnNode=True)
        # This is adapted from https://www.slicer.org/wiki/Documentation/4.3/Modules/VolumeRendering
        # The effect is that the volume rendering changes when the segmentation array changes
        slicer_logic = slicer.modules.volumerendering.logic()
        self.displayNode = slicer_logic.CreateVolumeRenderingDisplayNode()
        slicer.mrmlScene.AddNode(self.displayNode)
        self.displayNode.UnRegister(slicer_logic)
        slicer_logic.UpdateDisplayNodeFromVolumeNode(self.displayNode, self.label_volumeNode)
        self.label_volumeNode.AddAndObserveDisplayNodeID(self.displayNode.GetID())
        self.voxel_array = slicer.util.arrayFromVolume(self.label_volumeNode)

    def register_visual_change(self):
        """ Method to call after changing self.voxel_array so that the visualizations update """
        self.label_volumeNode.Modified()  # Updates the visualizations
        self.displayNode.Modified()

