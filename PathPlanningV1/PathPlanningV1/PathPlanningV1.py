# Standard library imports
import logging
import os
from typing import Annotated, Optional

# Third-party library imports
import numpy as np
import vtk
import SimpleITK as sitk
import sitkUtils

# Slicer-specific imports
import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)
from slicer import vtkMRMLScalarVolumeNode, vtkMRMLLabelMapVolumeNode, vtkMRMLMarkupsFiducialNode


# Debugging and logging setup
log_filename = "test_results.log"
log_filepath = os.path.join(os.getcwd(), log_filename)

# Print current directory and check if it's writable
print(f"Current directory: {os.getcwd()}")
print(f"Directory is writable: {os.access(os.getcwd(), os.W_OK)}")

# Try to create a file directly to ensure directory is writable
try:
    with open(log_filepath, 'w') as test_file:
        test_file.write("Testing file creation.\n")
    print(f"Successfully created and wrote to {log_filepath}.")
except Exception as e:
    print(f"Failed to create or write to {log_filepath}: {e}")

# Remove the test file
if os.path.exists(log_filepath):
    os.remove(log_filepath)

try:
    # Configure logging
    logging.basicConfig(
        level=logging.DEBUG,  # Set to DEBUG to capture all messages
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_filepath, mode='w'),  # 'w' mode to overwrite on each run
            logging.StreamHandler()
        ]
    )
except Exception as e:
    print(f"Failed to set up logging: {e}")


#
# PathPlanningV1
#


class PathPlanningV1(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        # Initialize the parent class
        ScriptedLoadableModule.__init__(self, parent)

        # Set the module's title
        self.parent.title = _("PathPlanningV1") 

        # Set the categories this module belongs to (for organization within the software)
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]

        # Define any dependencies for this module (empty in this case)
        self.parent.dependencies = [] 

         # List contributors to this module
        self.parent.contributors = ["Farhana Moosa (King's College London), Rachel Sparks (King's College London')"] 

        # TODO: update with short description of the module and a link to online module documentation
        # _() function marks text as translatable to other languages
        self.parent.helpText = _("""
        This is an example of scripted loadable module bundled in an extension.
        See more information in <a href="https://github.com/organization/projectname#PathPlanningV1">module documentation</a>.
        """)

        self.parent.acknowledgementText = _("""
        This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
        and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
        """)


@parameterNodeWrapper
class PathPlanningV1ParameterNode:
     """
     The parameters needed by the module.

     inputTargetVolume - The label map the trajectory must be inside
     inputCriticalVolume - The label map the trajectory must avoid
     inputEntryFiducials - Fiducials cotaining potential entry points
     inputTargetFiducials - Fiducials containing potential target points
     lengthThreshold - The value above which to exclude trajectories
     outputFiducials - Fiducials containing output points of target and entry pairs
     """

     inputTargetVolume: vtkMRMLLabelMapVolumeNode
     inputCriticalVolume: vtkMRMLLabelMapVolumeNode
     inputEntryFiducials: vtkMRMLMarkupsFiducialNode
     inputTargetFiducials: vtkMRMLMarkupsFiducialNode
     lengthThreshold: Annotated[float, WithinRange(0, 200)] = 55
     outputFiducials: vtkMRMLMarkupsFiducialNode


#
# PathPlanningV1Widget
#


class PathPlanningV1Widget(ScriptedLoadableModuleWidget, VTKObservationMixin):
     """Uses ScriptedLoadableModuleWidget base class, available at:
     https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
     """

     def __init__(self, parent=None) -> None:
         """Called when the user opens the module the first time and the widget is initialized."""
         ScriptedLoadableModuleWidget.__init__(self, parent)
         VTKObservationMixin.__init__(self)  # needed for parameter node observation
         self.logic = None
         self._parameterNode = None
         self._parameterNodeGuiTag = None

     def setup(self) -> None:
         """Called when the user opens the module the first time and the widget is initialized."""
         ScriptedLoadableModuleWidget.setup(self)

         # Load widget from .ui file (created by Qt Designer).
         # Additional widgets can be instantiated manually and added to self.layout.
         uiWidget = slicer.util.loadUI(self.resourcePath("UI/PathPlanningV1.ui"))
         self.layout.addWidget(uiWidget)
         self.ui = slicer.util.childWidgetVariables(uiWidget)

         # Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
         # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
         # "setMRMLScene(vtkMRMLScene*)" slot.
         uiWidget.setMRMLScene(slicer.mrmlScene)

         # Create logic class. Logic implements all computations that should be possible to run
         # in batch mode, without a graphical user interface.
         self.logic = PathPlanningV1Logic()

         # Connections

         # These connections ensure that we update parameter node when scene is closed
         self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
         self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

         # Buttons
         self.ui.applyButton.connect("clicked(bool)", self.onApplyButton)

         # Make sure parameter node is initialized (needed for module reload)
         self.initializeParameterNode()

         # Connect the slider value change to a method to update the parameter node
         self.ui.lengthThresholdSlider.valueChanged.connect(self.updateLengthThreshold)

     def cleanup(self) -> None:
         """Called when the application closes and the module widget is destroyed."""
         self.removeObservers()

     def enter(self) -> None:
         """Called each time the user opens this module."""
         # Make sure parameter node exists and observed
         self.initializeParameterNode()

     def exit(self) -> None:
         """Called each time the user opens a different module."""
         # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
         if self._parameterNode:
             self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
             self._parameterNodeGuiTag = None
             self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)

     def onSceneStartClose(self, caller, event) -> None:
         """Called just before the scene is closed."""
         # Parameter node will be reset, do not use it anymore
         self.setParameterNode(None)

     def onSceneEndClose(self, caller, event) -> None:
         """Called just after the scene is closed."""
         # If this module is shown while the scene is closed then recreate a new parameter node immediately
         if self.parent.isEntered:
             self.initializeParameterNode()

     def initializeParameterNode(self) -> None:
         """Ensure parameter node exists and observed."""
         # Parameter node stores all user choices in parameter values, node selections, etc.
         # so that when the scene is saved and reloaded, these settings are restored.

         self.setParameterNode(self.logic.getParameterNode())

         # Select default input nodes if nothing is selected yet to save a few clicks for the user
         if not self._parameterNode.inputTargetVolume:
             firstVolumeNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLLabelMapVolumeNode")
             if firstVolumeNode:
                 self._parameterNode.inputTargetVolume = firstVolumeNode

         if not self._parameterNode.inputCriticalVolume:
             firstVolumeNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLLabelMapVolumeNode")
             if firstVolumeNode:
                 self._parameterNode.inputCriticalVolume = firstVolumeNode

         if not self._parameterNode.inputTargetFiducials:
             firstFiducialNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLMarkupsFiducialNode")
             if firstFiducialNode:
                 self._parameterNode.inputTargetFiducials = firstFiducialNode

         if not self._parameterNode.inputEntryFiducials:
             firstFiducialNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLMarkupsFiducialNode")
             if firstFiducialNode:
                 self._parameterNode.inputEntryFiducials = firstFiducialNode

     def setParameterNode(self, inputParameterNode: Optional[PathPlanningV1ParameterNode]) -> None:
         """
         Set and observe parameter node.
         Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
         """

         if self._parameterNode:
             self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
             self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
         self._parameterNode = inputParameterNode
         if self._parameterNode:
             # Note: in the .ui file, a Qt dynamic property called "SlicerParameterName" is set on each
             # ui element that needs connection.
             self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
             self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
             self._checkCanApply()

     def _checkCanApply(self, caller=None, event=None) -> None:
         if self._parameterNode and self._parameterNode.inputTargetVolume and self._parameterNode.inputCriticalVolume and self._parameterNode.inputEntryFiducials and self._parameterNode.inputTargetFiducials:
             self.ui.applyButton.toolTip = _("Compute Trajectory")
             self.ui.applyButton.enabled = True
         else:
             self.ui.applyButton.toolTip = _("Select all input nodes")
             self.ui.applyButton.enabled = False

     def updateLengthThreshold(self, value):
         """Update the length threshold parameter in the parameter node."""
         if self._parameterNode:
            self._parameterNode.lengthThreshold = value

     def onApplyButton(self) -> None:
         """Run path planning logic when user clicks "Apply" button."""
         try:
             # Set class parameters
             self.logic.SetEntryPoints(self.ui.inputEntryFiducialSelector.currentNode())
             self.logic.SetTargetPoints(self.ui.inputTargetFiducialSelector.currentNode())
             self.logic.SetOutputPoints(self.ui.outputFiducialSelector.currentNode())
             self.logic.SetInputTargetImage(self.ui.inputTargetVolumeSelector.currentNode())
             self.logic.SetInputCriticalVolume(self.ui.inputCriticalVolumeSelector.currentNode())
             self.logic.SetLengthThreshold(self._parameterNode.lengthThreshold)

             # Run the code. Return false if the code did not run properly
             complete = self.logic.run()

             # Print out an error message if the code returned false
             if not complete:
                 logging.error('Error encountered: failed to execute path planning algorithm')
                 slicer.util.errorDisplay('Error encountered: failed to execute path planning algorithm')

         except ValueError as e:
              #error_message = f"Error during setup: {e}"
              #logging.error(error_message)
              slicer.util.errorDisplay(f"Error during setup: {e}")
         except Exception as e:
              #error_message = f"Unexpected error: {e}"
              #logging.error(error_message)
              slicer.util.errorDisplay(f"Unexpected error: {e}")


#
# PathPlanningV1Logic
#

class PathPlanningV1Logic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""

        # Call the constructor of the parent class to ensure proper initialization
        ScriptedLoadableModuleLogic.__init__(self)
        self.myEntries = None
        self.myTargets = None
        self.myTargetImage = None
        self.myCriticalVolume = None
        self.myOutputs = None
        self.lengthThreshold = None

    def getParameterNode(self):
        return PathPlanningV1ParameterNode(super().getParameterNode())

    def SetEntryPoints(self, entryNode):
        valid, message = self.hasFiducials(entryNode)
        if valid:
            self.myEntries = entryNode
        else:
            raise ValueError(f'SetEntryPoints failed: {message}')

    def SetTargetPoints(self, targetNode):
        valid, message = self.hasFiducials(targetNode)
        if valid:
            self.myTargets = targetNode
        else:
            raise ValueError(f'SetTargetPoints failed: {message}')

    def SetInputTargetImage(self, imageNode):
        valid, message = self.hasImageData(imageNode)
        if valid:
            self.myTargetImage = imageNode
        else:
            raise ValueError(f'SetInputTargetImage failed: {message}')

    def SetInputCriticalVolume(self, criticalVolume):
        valid, message = self.hasImageData(criticalVolume)
        if valid:
            self.myCriticalVolume = criticalVolume
        else:
            raise ValueError(f'SetInputCriticalVolume failed: {message}')

    def SetOutputPoints(self, outputNode):
        if outputNode is None:
            raise ValueError("SetOutputPoints failed: No Output Fiducials selected.")
        if self.myEntries is not None and outputNode.GetName() == self.myEntries.GetName():
            raise ValueError("SetOutputPoints failed: Output fiducials node name conflicts with entry points node name.")
        if self.myTargets is not None and outputNode.GetName() == self.myTargets.GetName():
            raise ValueError("SetOutputPoints failed: Output fiducials node name conflicts with target points node name.")
        self.myOutputs = outputNode

    def SetLengthThreshold(self, threshold):
        if threshold <= 0:
            raise ValueError("SetLengthThreshold failed: Length threshold must be greater than zero.")
        self.lengthThreshold = threshold

    def hasImageData(self, volumeNode):
        """This method returns true if the passed in volume node has valid image data."""
        if not volumeNode:
            return False, 'Volume node does not exist'

        if volumeNode.GetImageData() is None:
            return False, 'No image data in volume node'

        return True, ''

    def hasFiducials(self, fiducialNode):
        """This method returns true if the passed-in fiducial node has at least one fiducial point."""
        if not fiducialNode:
            return False, 'Fiducial node does not exist'

        if fiducialNode.GetNumberOfControlPoints() == 0:
            return False, 'Fiducial node is empty'

        return True, ''

    def isValidInputOutputData(self, inputTargetVolumeNode, inputTargetFiducialsNode, inputEntryFiducialsNodes, outputFiducialsNode):
        """Validates if the output is not the same as input"""
        if not inputTargetVolumeNode:
          logging.debug('isValidInputOutputData failed: no input target volume node defined')
          return False
        if not inputTargetFiducialsNode:
          logging.debug('isValidInputOutputData failed: no input target fiducials node defined')
          return False
        if not inputEntryFiducialsNodes:
          logging.debug('isValidInputOutputData failed: no input entry fiducials node defined')
          return False
        if not outputFiducialsNode:
          logging.debug('isValidInputOutputData failed: no output fiducials node defined')
          return False
        if inputTargetFiducialsNode.GetID()==outputFiducialsNode.GetID():
          logging.debug('isValidInputOutputData failed: input and output fiducial nodes are the same. Create a new output to avoid this error.')
          return False

        return True

    def run(self):
        """
        Run the path planning algorithm.
        """

        if not self.isValidInputOutputData(self.myTargetImage, self.myTargets, self.myEntries, self.myOutputs):
          slicer.util.errorDisplay('Not all inputs are set.')
          return False
        if not self.hasImageData(self.myTargetImage):
            raise ValueError("Input target volume is not appropriatelly defined.")

        import time
        startTime = time.time()
        logging.info("Processing started")
        
        hardConstraints = HardConstraints()
        hardConstraints.checkIfPointInTarget(self.myTargetImage, self.myTargets, self.myOutputs)
        allTrajectories = hardConstraints.generateTrajectoriesWithoutIntersection(self.myEntries,self.myOutputs,self.myCriticalVolume)
        filteredTrajectories = hardConstraints.filterTrajectoriesByLength(allTrajectories, self.lengthThreshold)

        softConstraints = SoftConstraints()
        softConstraints.computeDistanceImageFromLabelMap(self.myCriticalVolume)
        minimumDistances = softConstraints.findDistanceToClosestVoxelInsideLabelMap(softConstraints.outputDistanceMap,filteredTrajectories)
        softConstraints.findBestTrajectory(filteredTrajectories,minimumDistances)
        softConstraints.displayBestTrajectory()
        stopTime = time.time()
        logging.info(f"Processing completed in {stopTime-startTime:.2f} seconds")
        return True


class HardConstraints():


    def checkIfPointInTarget(self, inputVolume, inputFiducials, outputFiducials):
        """
        Verifies if target points are within a specified target region of a label map volume and ensures they
        are not on the edge of the target region. 

        This method performs the following steps:
        1. Transforms fiducial points from RAS (Right-Anterior-Superior) coordinates to IJK (image voxel) coordinates.
        2. Checks if the transformed points fall within the bounds of the input label map.
        3. Ensures that the points are within the target region, defined by voxels with a value of 1.
        4. Excludes points that are on the edge of the target region by verifying the value of neighboring voxels.
        5. Adds valid points to the output fiducials.

        Parameters:
        - inputVolume (vtkMRMLLabelMapVolumeNode): The label map containing the target region.
        - inputFiducials (vtkMRMLMarkupsFiducialNode): target points to be checked.
        - outputFiducials (vtkMRMLMarkupsFiducialNode): The fiducial points that are within the target region 
          and not on the edge are added to this node in RAS coordinates.
        """

        try:
            # Remove all existing control points from the output fiducials
            outputFiducials.RemoveAllControlPoints()

            # Compute the transformation matrix from RAS to IJK coordinates
            rasToIjkMatrix = vtk.vtkMatrix4x4()
            inputVolume.GetRASToIJKMatrix(rasToIjkMatrix)

            # Prepare to access voxel data directly from the image
            imageData = inputVolume.GetImageData()
            dims = imageData.GetDimensions()

            # Define neighbor offsets for checking edge condition
            neighborOffsets = [
                (-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)
            ]

            # Iterate through each control point in the input fiducials
            for i in range(inputFiducials.GetNumberOfControlPoints()):
                try:
                    # Initialize a point variable to hold the RAS coordinates
                    point = [0, 0, 0]
                    inputFiducials.GetNthControlPointPosition(i, point)
                
                    # Convert the RAS coordinates to IJK coordinates
                    # Initialize pointIjk with homogeneous coordinates
                    pointIjk = [0, 0, 0, 1]
                    rasToIjkMatrix.MultiplyPoint(np.append(point, 1.0), pointIjk)
                    pointIjk = [int(round(c)) for c in pointIjk[0:3]]

                    # Check if the IJK coordinates are within the image dimensions
                    if (0 <= pointIjk[0] < dims[0] and 0 <= pointIjk[1] < dims[1] and 0 <= pointIjk[2] < dims[2]):
                        # Get the voxel value at the specified IJK coordinates
                        pixelValue = imageData.GetScalarComponentAsDouble(pointIjk[0], pointIjk[1], pointIjk[2], 0)

                        # Check if the point is within the target region and not on the edge
                        if pixelValue == 1:
                            isEdgePoint = False
                            for offset in neighborOffsets:
                                neighborIjk = [pointIjk[0] + offset[0], pointIjk[1] + offset[1], pointIjk[2] + offset[2]]
                                if not (0 <= neighborIjk[0] < dims[0] and 0 <= neighborIjk[1] < dims[1] and 0 <= neighborIjk[2] < dims[2]):
                                    isEdgePoint = True
                                    break
                                neighborPixelValue = imageData.GetScalarComponentAsDouble(neighborIjk[0], neighborIjk[1], neighborIjk[2], 0)
                                if neighborPixelValue != 1:
                                    isEdgePoint = True
                                    break
                            
                            if not isEdgePoint:
                                outputFiducials.AddControlPoint(point[0], point[1], point[2])
                    else:
                        logging.error("One or more points are out of the image bounds.")
                        return

                except Exception as e:
                    message = f"Error processing point {point}: {str(e)}"
                    logging.error(message)
                    return

        except Exception as e:
            message = f"Error in checkIfPointInTarget: {str(e)}"
            logging.error(message)
            raise

    def generateTrajectoriesWithoutIntersection(self, inputEntryFiducials, outputTargetFiducials, inputCriticalVolume):
        """
        Generate trajectories between entry and target points that do not intersect the critical volume.

        Parameters:
        - inputEntryFiducials: vtkMRMLMarkupsFiducialNode containing potential entry points.
        - outputTargetFiducials: vtkMRMLMarkupsFiducialNode containing target points inside target.
        - inputCriticalVolume: vtkMRMLLabelMapVolumeNode representing the critical volume to avoid.

        Returns:
        - List of tuples, where each tuple contains two points (entry and target) in RAS coordinates.
        """
        allTrajectories = [] # Initialize an empty list to store all trajectories

        # Set up Marching Cubes algorithm to generate a surface mesh from the critical volume
        marchingCubes = vtk.vtkMarchingCubes()
        marchingCubes.SetValue(0, 0.5) # Define the isosurface value for the mesh generation
        marchingCubes.SetInputData(inputCriticalVolume.GetImageData()) # Set the input data for Marching Cubes
        marchingCubes.Update()  # Execute the algorithm to create the mesh
        outputPolyData = marchingCubes.GetOutput() # Get the resulting surface mesh

        # Set up Oriented Bounding Box (OBB) tree for efficient intersection testing
        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(outputPolyData) # Set the polydata for the OBB tree
        obbTree.BuildLocator()  # Build the OBB tree for the given polydata

        # Compute the RAS to IJK transformation matrix
        volumeRasToIjk = vtk.vtkMatrix4x4()
        inputCriticalVolume.GetRASToIJKMatrix(volumeRasToIjk)

        # Compute the IJK to RAS transformation matrix
        volumeIjkToRas = vtk.vtkMatrix4x4()
        inputCriticalVolume.GetIJKToRASMatrix(volumeIjkToRas)

        for i in range(inputEntryFiducials.GetNumberOfControlPoints()):
            entryRasPoint = [0, 0, 0]
            inputEntryFiducials.GetNthControlPointPosition(i, entryRasPoint)
            entryRasPoint.append(1.0)  # Add homogeneous coordinate

            # Transform entry point to IJK space
            entryIjkPoint = [0, 0, 0, 1]
            volumeRasToIjk.MultiplyPoint(entryRasPoint, entryIjkPoint)
            entryIjkPoint = [entryIjkPoint[0], entryIjkPoint[1], entryIjkPoint[2]]

            for j in range(outputTargetFiducials.GetNumberOfControlPoints()):
                targetRasPoint = [0, 0, 0]
                outputTargetFiducials.GetNthControlPointPosition(j, targetRasPoint)
                targetRasPoint.append(1.0)  # Add homogeneous coordinate

                # Transform target point to IJK space
                targetIjkPoint = [0, 0, 0, 1]
                volumeRasToIjk.MultiplyPoint(targetRasPoint, targetIjkPoint)
                targetIjkPoint = [targetIjkPoint[0], targetIjkPoint[1], targetIjkPoint[2]]

                line = vtk.vtkLineSource()
                line.SetPoint1(entryIjkPoint)
                line.SetPoint2(targetIjkPoint)
                line.Update()

                intersection = obbTree.IntersectWithLine(line.GetPoint1(), line.GetPoint2(), None, None)
                if intersection == 0:
                    # Transform points back to RAS space before appending
                    entryRasPointBack = [0, 0, 0, 1]
                    volumeIjkToRas.MultiplyPoint(entryIjkPoint + [1.0], entryRasPointBack)
                    entryRasPointBack = entryRasPointBack[:3]

                    targetRasPointBack = [0, 0, 0, 1]
                    volumeIjkToRas.MultiplyPoint(targetIjkPoint + [1.0], targetRasPointBack)
                    targetRasPointBack = targetRasPointBack[:3]

                    allTrajectories.append((entryRasPointBack, targetRasPointBack))
        print(len(allTrajectories))
       
        return allTrajectories

    def filterTrajectoriesByLength(self, allTrajectories, lengthThreshold):
        """
        Filter out trajectories longer than the given length threshold.
        """

        filteredTrajectories = []  # Initialize an empty list to store the filtered trajectories

        # Loop through each trajectory in the input list
        for trajectory in allTrajectories:
            entryPoint = np.array(trajectory[0]) # Convert the entry point of the trajectory to a numpy array
            targetPoint = np.array(trajectory[1])  # Convert the target point of the trajectory to a numpy array

            # Calculate the Euclidean distance (length) between the entry point and the target point
            length = np.linalg.norm(entryPoint - targetPoint)

            # If the length of the trajectory is less than or equal to the specified threshold, add it to the filtered list
            if length <= lengthThreshold:
                filteredTrajectories.append(trajectory)
        print(len(filteredTrajectories))

        return filteredTrajectories


class SoftConstraints():
    
    def computeDistanceImageFromLabelMap(self,inputLabelMap):
        sitkInput = sitkUtils.PullVolumeFromSlicer(inputLabelMap)
        distanceFilter = sitk.DanielssonDistanceMapImageFilter() 
    
        sitkOutput = distanceFilter.Execute(sitkInput)
        self.outputDistanceMap = sitkUtils.PushVolumeToSlicer(sitkOutput, None, 'distanceMap')

    def findDistanceToClosestVoxelInsideLabelMap(self, outputDistanceMap, filteredTrajectories):
        minimumDistances = []
        numberOfPoints = 100

        # Pull the distance map from Slicer to SimpleITK
        sitkDistanceMap = sitkUtils.PullVolumeFromSlicer(outputDistanceMap)
        distanceArray = sitk.GetArrayFromImage(sitkDistanceMap)

        # Get the RAS to IJK transformation matrix
        volumeNode = slicer.util.getNode(outputDistanceMap.GetName())
        rasToIjkMatrix = vtk.vtkMatrix4x4()
        volumeNode.GetRASToIJKMatrix(rasToIjkMatrix)

        for trajectory in filteredTrajectories:
            startLine = np.array(trajectory[0])
            endLine = np.array(trajectory[1])

            # Convert startLine and endLine to homogeneous coordinates
            startLineHomogeneous = np.append(startLine, 1)
            endLineHomogeneous = np.append(endLine, 1)
        
            linePoints = np.linspace(startLineHomogeneous, endLineHomogeneous, numberOfPoints)


            # Transform RAS points to IJK indices using the matrix
            linePointsIjk = []
            for point in linePoints:
                pointIjk = [0, 0, 0, 1]
                rasToIjkMatrix.MultiplyPoint(point, pointIjk)
                pointIjk = [int(round(coord)) for coord in pointIjk[:3]]
                linePointsIjk.append(pointIjk)

            # Ensure indices are within bounds
            linePointsIjk = np.clip(linePointsIjk, 0, np.array(distanceArray.shape) - 1)

            # Sample the distance map at the line points
            distances = distanceArray[tuple(np.array(linePointsIjk).T)]

            # Find the minimum distance
            minDistance = np.min(distances)

            # Append the minimum distance to the result list
            minimumDistances.append(minDistance)

        return minimumDistances

    def findBestTrajectory(self, filteredTrajectories,minimumDistances):
        # Find index which corresponds to maximun distance
        indexMax = minimumDistances.index(max(minimumDistances))
        entryPoint = filteredTrajectories[indexMax][0]
        targetPoint = filteredTrajectories[indexMax][1]
        markupsNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        markupsNode.AddControlPoint(entryPoint)
        markupsNode.AddControlPoint(targetPoint)
        markupsNode.SetName('Trajectory Points')
        self.Trajectory = (entryPoint, targetPoint)

    def displayBestTrajectory(self):
        lineNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsLineNode','Trajectory Display')
        lineNode.AddControlPoint(self.Trajectory[0])
        lineNode.AddControlPoint(self.Trajectory[1])   


#
# PathPlanningV1Test
#


class PathPlanningV1Test(ScriptedLoadableModuleTest):
    """
    This is the test case for the PathPlanningV1 module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """Reset the state."""
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_checkIfPointInTarget()
        self.test_generateTrajectoriesWithoutIntersection()
        self.test_filterTrajectoriesByLength()
        self.test_computeDistanceImageFromLabelMap()
        self.test_findDistanceToClosestVoxelInsideLabelMap()
        self.test_findBestTrajectory ()
    def test_checkIfPointInTarget(self):
        """
        Test the checkIfPointInTarget function with a mock inputVolume.
        This function tests three scenarios:
        1. A point that is known to be inside the target region.
        2. A point that is known to be outside the target region, but within image bounds.
        3. A point that is on the boundary of the target region.
        """
        # Create a mock volume node with image data containing a simple target region
        volumeNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLabelMapVolumeNode')
        imageData = vtk.vtkImageData()
        imageData.SetDimensions(10, 10, 10)
        imageData.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)

        # Fill the image data with zeros
        for z in range(10):
            for y in range(10):
                for x in range(10):
                    imageData.SetScalarComponentFromDouble(x, y, z, 0, 0)

        # Define a larger target region
        for z in range(4, 7):  # Extending the region to include multiple voxels
            for y in range(4, 7):
                for x in range(4, 7):
                    imageData.SetScalarComponentFromDouble(x, y, z, 0, 1)

        volumeNode.SetAndObserveImageData(imageData)
        rasToIjkMatrix = vtk.vtkMatrix4x4()
        volumeNode.GetRASToIJKMatrix(rasToIjkMatrix)

        # Create input fiducial nodes for testing
        inputFiducials = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        inputFiducials.AddControlPoint([5.0, 5.0, 5.0])  # Point inside the target
        inputFiducials.AddControlPoint([0.0, 0.0, 0.0])  # Point outside the target but within bounds
        inputFiducials.AddControlPoint([4.0, 3.0, 4.0])  # Point on the boundary of the target

        # Create an output fiducial node
        outputFiducials = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')

        # Instantiate the HardConstraints class
        hardConstraints = HardConstraints()

        # Call the method under test
        hardConstraints.checkIfPointInTarget(volumeNode, inputFiducials, outputFiducials)

        # Verify the output
        insideTestPassed = False
        outsideWithinBoundsTestPassed = True

        print("Output Fiducials:")
        for i in range(outputFiducials.GetNumberOfControlPoints()):
            point = [0, 0, 0]
            outputFiducials.GetNthControlPointPosition(i, point)
            print(f"Point {i}: {point}")
            if point == [5.0, 5.0, 5.0]:
                insideTestPassed = True
            elif point == [0.0, 0.0, 0.0]:
                outsideWithinBoundsTestPassed = False

        # Report test results
        if insideTestPassed and outsideWithinBoundsTestPassed:
            print("test_checkIfPointInTarget passed: Inside point correctly identified as inside, outside points correctly identified as outside.")
        else:
            print("test_checkIfPointInTarget failed:")
            if not insideTestPassed:
                logging.error("- Inside point was not correctly identified as inside.")
            if not outsideWithinBoundsTestPassed:
                logging.error("- Outside within bounds point was incorrectly identified as inside.")

    def test_generateTrajectoriesWithoutIntersection(self):
        """
        Test the generateTrajectoriesWithoutIntersection function.
        """
        hardConstraints = HardConstraints()

        # Create a mock critical volume node
        inputCriticalVolume = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLabelMapVolumeNode')
        imageData = vtk.vtkImageData()
        imageData.SetDimensions(10, 10, 10)
        imageData.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)
        
        # Fill the image data with zeros and set some voxels to represent the critical volume
        for z in range(10):
            for y in range(10):
                for x in range(10):
                    imageData.SetScalarComponentFromDouble(x, y, z, 0, 0)
        imageData.SetScalarComponentFromDouble(5, 5, 5, 0, 1)  # Critical point

        inputCriticalVolume.SetAndObserveImageData(imageData)

        # Create mock entry and target fiducials
        inputEntryFiducials = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
        outputTargetFiducials = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')

        # Add control points that will generate trajectories through and avoiding the critical volume
        inputEntryFiducials.AddControlPoint([0, 0, 0])
        inputEntryFiducials.AddControlPoint([3, 3, 3])
        outputTargetFiducials.AddControlPoint([4, 4, 4])
        outputTargetFiducials.AddControlPoint([6, 6, 6])

        # Call the method
        generatedTrajectories = hardConstraints.generateTrajectoriesWithoutIntersection(inputEntryFiducials, outputTargetFiducials, inputCriticalVolume)

        # Expected trajectories (one should be excluded because it intersects the critical volume)
        expectedTrajectories = [
            ([0, 0, 0], [4, 4, 4]),  # This trajectory does not intersect the critical volume
            ([3, 3, 3], [4, 4, 4])   # This trajectory does not intersect the critical volume
        ]

        self.assertEqual(len(generatedTrajectories), len(expectedTrajectories))
        for traj in generatedTrajectories:
            self.assertIn(traj, expectedTrajectories)

        if len(generatedTrajectories) == len(expectedTrajectories) and all(traj in expectedTrajectories for traj in generatedTrajectories):
            logging.info(f"test_generateTrajectoriesWithoutIntersection passed: Generated trajectories {generatedTrajectories} match expected {expectedTrajectories}.")
        else:
            logging.error(f"test_generateTrajectoriesWithoutIntersection failed: Generated trajectories {generatedTrajectories} do not match expected {expectedTrajectories}.")

    def test_filterTrajectoriesByLength(self):
        """
        Test the filterTrajectoriesByLength function.
        """
        hardConstraints = HardConstraints()

        allTrajectories = [
            ([0, 0, 0], [3, 4, 0]),  # length is exactly 5
            ([0, 0, 0], [1, 1, 1]),  # length is less than 5
            ([0, 0, 0], [10, 10, 10]) # length is greater than 5
        ]
        lengthThreshold = 5

        expectedFilteredTrajectories = [
            ([0, 0, 0], [3, 4, 0]),
            ([0, 0, 0], [1, 1, 1])
        ]

        filteredTrajectories = hardConstraints.filterTrajectoriesByLength(allTrajectories, lengthThreshold)
        self.assertEqual(len(filteredTrajectories), len(expectedFilteredTrajectories))
        for traj in filteredTrajectories:
            self.assertIn(traj, expectedFilteredTrajectories)

        if len(filteredTrajectories) == len(expectedFilteredTrajectories) and all(traj in expectedFilteredTrajectories for traj in filteredTrajectories):
            logging.info(f"test_filterTrajectoriesByLength passed: Tested trajectories {allTrajectories} with threshold {lengthThreshold}. Expected {expectedFilteredTrajectories}, got {filteredTrajectories}.")
        else:
            logging.error(f"test_filterTrajectoriesByLength failed: Tested trajectories {allTrajectories} with threshold {lengthThreshold}. Expected {expectedFilteredTrajectories}, got {filteredTrajectories}.")

    def test_computeDistanceImageFromLabelMap(self):
        """
        Test the computeDistanceImageFromLabelMap function.
        """
        # Create a mock label map volume node
        labelMapVolumeNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLabelMapVolumeNode')
        imageData = vtk.vtkImageData()
        imageData.SetDimensions(5, 5, 5)
        imageData.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)

        # Fill the image data with zeros and set a voxel value
        for z in range(5):
            for y in range(5):
                for x in range(5):
                    imageData.SetScalarComponentFromDouble(x, y, z, 0, 0)
        imageData.SetScalarComponentFromDouble(2, 2, 2, 0, 1)

        labelMapVolumeNode.SetAndObserveImageData(imageData)

        # Call the method under test
        softConstraints=SoftConstraints()
        softConstraints.computeDistanceImageFromLabelMap(labelMapVolumeNode)

        # Pull the distance map back to SimpleITK for verification
        distanceMap = sitkUtils.PullVolumeFromSlicer(softConstraints.outputDistanceMap)
        distanceArray = sitk.GetArrayFromImage(distanceMap)

        # Calculate the expected distance array
        expectedDistanceArray = np.zeros((5, 5, 5))
        center = np.array([2, 2, 2])
        for z in range(5):
            for y in range(5):
                for x in range(5):
                    expectedDistanceArray[z, y, x] = np.linalg.norm(np.array([x, y, z]) - center)

        # Verify the output
        try:
            self.assertTrue(np.allclose(distanceArray, expectedDistanceArray, atol=1e-5))
            logging.info("test_computeDistanceImageFromLabelMap passed: Distance map computed correctly for input label map.")
        except AssertionError:
            logging.error(f"test_computeDistanceImageFromLabelMap failed: Expected distance array {expectedDistanceArray}, but got {distanceArray}.")

    def test_findDistanceToClosestVoxelInsideLabelMap(self):
        """
        Test the findDistanceToClosestVoxelInsideLabelMap function.
        """
        constraints = SoftConstraints()

        # Create a mock label map volume node with a critical structure
        labelMapVolumeNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLabelMapVolumeNode')
        imageData = vtk.vtkImageData()
        imageData.SetDimensions(5, 5, 5)
        imageData.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)

        # Fill the image data with zeros and set a voxel value to 1 for the critical structure
        for z in range(5):
            for y in range(5):
                for x in range(5):
                    imageData.SetScalarComponentFromDouble(x, y, z, 0, 0)
        imageData.SetScalarComponentFromDouble(4, 4, 4, 0, 1)

        labelMapVolumeNode.SetAndObserveImageData(imageData)

        # Create a distance map from the label map volume
        sitkLabelMap = sitkUtils.PullVolumeFromSlicer(labelMapVolumeNode)
        distanceFilter = sitk.DanielssonDistanceMapImageFilter()
        sitkDistanceMap = distanceFilter.Execute(sitkLabelMap)
        outputDistanceMap = sitkUtils.PushVolumeToSlicer(sitkDistanceMap, None, 'distanceMap')

        # Mock trajectories
        filteredTrajectories = [
            ([0, 0, 0], [4, 4, 4]),  # A trajectory across the volume passing through critical structure
            ([1, 1, 1], [3, 3, 3]),  # Another trajectory not passing through critical structure
            ([0, 0, 0], [1, 1, 1]),  # A short trajectory not passing through critical structure
            ([2, 2, 2], [4, 4, 4]),  # A trajectory starting from the center passing through critical structure
        ]

        # Calculate the expected distances
        distanceArray = sitk.GetArrayFromImage(sitkDistanceMap)

        def calculate_expected_distance(start, end):
            start = np.array(start)
            end = np.array(end)
            line_points = np.linspace(start, end, 100)
            distances = [
                distanceArray[int(round(pt[2])), int(round(pt[1])), int(round(pt[0]))]
                for pt in line_points
            ]
            return np.min(distances)

        expectedDistances = [
            calculate_expected_distance([0, 0, 0], [4, 4, 4]),
            calculate_expected_distance([1, 1, 1], [3, 3, 3]),
            calculate_expected_distance([0, 0, 0], [1, 1, 1]),
            calculate_expected_distance([2, 2, 2], [4, 4, 4]),
        ]

        # Call the function
        minDistances = constraints.findDistanceToClosestVoxelInsideLabelMap(outputDistanceMap, filteredTrajectories)
        
        # Check if computed distances match expected distances
        for computed, expected in zip(minDistances, expectedDistances):
            self.assertAlmostEqual(computed, expected, places=5)
            if np.isclose(computed, expected, atol=1e-5):
                logging.info(f"test_findDistanceToClosestVoxelInsideLabelMap passed: Computed distance {computed} matches expected distance {expected}.")
            else:
                logging.error(f"test_findDistanceToClosestVoxelInsideLabelMap: Computed distance {computed} does not match expected distance {expected}.")


    def test_findBestTrajectory(self):
        """
        Test the findBestTrajectory function.
        """
        softConstraints = SoftConstraints()

        filteredTrajectories = [
            ([0, 0, 0], [1, 1, 1]),
            ([1, 1, 1], [2, 2, 2]),
            ([2, 2, 2], [3, 3, 3]),
            ([3, 3, 3], [4, 4, 4])
        ]
        minimumDistances = [1.732, 3.464, 5.196, 6.928]  # Distances for the respective trajectories

        expectedEntryPoint = [3, 3, 3]
        expectedTargetPoint = [4, 4, 4]

        # Call the method
        softConstraints.findBestTrajectory(filteredTrajectories, minimumDistances)

        # Verify the result
        self.assertEqual(softConstraints.Trajectory[0], expectedEntryPoint)
        self.assertEqual(softConstraints.Trajectory[1], expectedTargetPoint)

        # Verify the markups node in slicer
        markupsNode = slicer.util.getNode('Trajectory Points')
        self.assertIsNotNone(markupsNode)
        entryPoint = [0, 0, 0]
        targetPoint = [0, 0, 0]
        markupsNode.GetNthControlPointPosition(0, entryPoint)
        markupsNode.GetNthControlPointPosition(1, targetPoint)
        self.assertEqual(entryPoint, expectedEntryPoint)
        self.assertEqual(targetPoint, expectedTargetPoint)

        if softConstraints.Trajectory == (expectedEntryPoint, expectedTargetPoint) and entryPoint == expectedEntryPoint and targetPoint == expectedTargetPoint:
            logging.info(f"test_findBestTrajectory passed: Tested trajectories {filteredTrajectories} with distances {minimumDistances}. Expected EntryPoint = {expectedEntryPoint}, TargetPoint = {expectedTargetPoint}, got EntryPoint = {entryPoint}, TargetPoint = {targetPoint}.")
        else:
            logging.error(f"test_findBestTrajectory failed: Tested trajectories {filteredTrajectories} with distances {minimumDistances}. Expected EntryPoint = {expectedEntryPoint}, TargetPoint = {expectedTargetPoint}, got EntryPoint = {entryPoint}, TargetPoint = {targetPoint}.")

