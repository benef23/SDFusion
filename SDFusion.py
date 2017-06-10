import adsk.core
import adsk.fusion
import traceback
#import os.path
import xml.etree.ElementTree as ET
import math
import xml.dom.minidom as DOM

## @package SDFusion
# This is an exporter for Autodesk Fusion 360 models to SDFormat.
#
# This can be loaded as an addon in Fusion 360.
# It exports all rigid groups of the robot model as links
# to STL and creates nodes in the SDF file for them.
# It creates SDF nodes for all joints of the robot model.
# Supported joint types are: "fixed", "revolute", and "ball".

## Global variable to make the Fusion 360 design object accessible
# for every function.
design = None

## Global variable to make the output file directory accessible for
# every function.
fileDir = "C:/Users/techtalentsVR1/Documents/roboy/fusion/SDFusion"

## Global variable to make the robot model name accessible for
# every function.
modelName = "Legs"

## Global variable to make the root occurrence accessible for
# every function.
rootOcc = None

## Global variable to specify if the exporter should export viaPoints.
exportViaPoints = True

## Global variable to specify the file name of the plugin loaded by the SDF.
# Only necessary if **exportViaPoints** is **True**.
pluginFileName = "libDummy.so"

## Global variable to specify the name of the plugin loaded by the SDF-
# Only necessary if **exportViaPoints** id **True**.
pluginName = "DummyPlugin"

## Global viaPoint

## Transforms a matrix from Fusion 360 to Gazebo.
#
# This transforms a matrix given in the Fusion 360 coordinate system
# to one in the Gazebo cooridnate system.
#
# @param self a matrix given wrt the Fusion 360 coordinate system
# @return the matrix wrt the Gazebo coordinate system
def gazeboMatrix(self):
    matrix = adsk.core.Matrix3D.create()
    matrix.setCell(1, 1, 0)
    matrix.setCell(1, 2, -1)
    matrix.setCell(2, 1, 1)
    matrix.setCell(2, 2, 0)
    self.transformBy(matrix)
    return self

## Converts three double values to string.
#
# This function converts three double values to a string separated by spaces.
#
# @param x the first double value
# @param y the second double value
# @param z the third double value
# @return the string of these values
def vectorToString(x, y, z):
    string = str(x) + " " + str(y) + " " + str(z)
    return string

## Builds SDF pose node from vector.
#
# This function builds the SDF pose node for every joint.
#
# @param vector the vector pointing to the origin of the joint.
# @return the SDF pose node
def sdfPoseVector(vector):
    pose = ET.Element("pose", frame="")
    # convert from cm (Fusion 360) to m (SI)
    x = 0.01 * vector.x
    y = 0.01 * vector.y
    z = 0.01 * vector.z
    pos = vectorToString(x, y, z)
    rot = vectorToString(0, 0, 0)
    pose.text = pos + " " + rot
    return pose
    
## Builds SDF pose node from matrix.
#
# This function builds the SDF pose node for every link.
#
# @param matrix the transformation matrix of the link
# @return the SDF pose node
def sdfPoseMatrix(matrix):
    pose = ET.Element("pose", frame="")
    # convert from cm (Fusion 360) to m (SI)
    trans = matrix.translation
    x = 0.01 * trans.x
    y = 0.01 * trans.y
    z = 0.01 * trans.z
    pos = vectorToString(x, y, z)
    # calculate roll pitch yaw from transformation matrix
    r11 = matrix.getCell(0, 0)
    r21 = matrix.getCell(1, 0)
    r31 = matrix.getCell(2, 0)
    r32 = matrix.getCell(2, 1)
    r33 = matrix.getCell(2, 2)
    pitch = math.atan2(-r31, math.sqrt(math.pow(r11, 2) + math.pow(r21, 2)))
    cp = math.cos(pitch)
    yaw = math.atan2(r21 / cp, r11 / cp)
    roll = math.atan2(r32 / cp, r33 / cp)
    rot = vectorToString(roll, pitch, yaw)
    pose.text = pos + " " + rot
    return pose

## Builds SDF inertial node from physical properties.
#
# This function builds the SDF inertial node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertial node
def sdfInertial(physics):
    inertial = ET.Element("inertial")
    # build pose node of COM
    com = physics.centerOfMass
    pose = sdfPoseVector(com)
    inertial.append(pose)
    # build mass node
    mass = ET.Element("mass")
    mass.text = str(physics.mass)
    inertial.append(mass)
    # build inertia node
    inertia = sdfInertia(physics)
    inertial.append(inertia)
    return inertial

## Builds SDF node for one moment of inertia.
#
# This helper function builds the SDF node for one moment of inertia.
#
# @param tag the tag of the XML node
# @param value the text of the XML node
# @return the SDF moment of inertia node
def sdfMom(tag, value):
    node = ET.Element(tag)
    # convert from kg/cm^2 (Fusion 360) to kg/m^2 (SI)
    node.text = str(0.0001 * value)
    return node

## Builds SDF inertia node from physical properties.
#
# This function builds the SDF inertia node for every link.
#
# @param physics the physical properties of a link
# @return the SDF inertia node
def sdfInertia(physics):
    inertia = ET.Element("inertia")
    (returnValue, xx, yy, zz, xy, yz, xz) = physics.getXYZMomentsOfInertia()
    inertia.append(sdfMom("ixx", xx))
    inertia.append(sdfMom("ixy", xy))
    inertia.append(sdfMom("ixz", xz))
    inertia.append(sdfMom("iyy", yy))
    inertia.append(sdfMom("iyz", yz))
    inertia.append(sdfMom("izz", zz))
    return inertia
    
## Builds SDF link node.
#
# This function builds the SDF link node for every link.
#
# @param lin the link to be exported
# @return the SDF link node
def linkSDF(lin):
    linkName = lin.component.name
    link = ET.Element("link", name=linkName)
    # build pose node
    matrix = gazeboMatrix(lin.transform)
    pose = sdfPoseMatrix(matrix)
    link.append(pose)
    # get physical properties of occurrence
    physics = lin.physicalProperties
    # build inertial node
    inertial = sdfInertial(physics)
    link.append(inertial)
    # build collision node
    collision = ET.Element("collision", name = linkName + "_collision")
    link.append(collision)
    # build geometry node
    geometry = ET.Element("geometry")
    collision.append(geometry)
    # build mesh node
    mesh = ET.Element("mesh")
    geometry.append(mesh)
    # build uri node
    uri = ET.Element("uri")
    global modelName
    uri.text = "model://" + modelName + "/meshes/" + linkName + ".stl"
    mesh.append(uri)
    # scale the mesh from mm to m
    scale = ET.Element("scale")
    scale.text = "0.001 0.001 0.001"
    mesh.append(scale)
    # build visual node (equal to collision node)
    visual = ET.Element("visual", name = linkName + "_visual")
    visual.append(geometry)
    link.append(visual)
    return link
    
## Builds SDF joint node.
#
# This function builds the SDF joint node for every joint type.
#
# @param joi the joint 
# @param name_parent the name of the parent link
# @param name_child the name of the child link
# @return the SDF joint node
def jointSDF(joi, name_parent, name_child):
    jointInfo = []
    jointType = ""
    jType = joi.jointMotion.jointType
    if jType == 0:
        jointType = "fixed"
    elif jType == 1:
        jointInfo = revoluteJoint(joi)
        jointType = "revolute"
    elif jType == 2:
        # not implemented
        jointType = ""
    elif jType == 3:
        # not implemented
        jointType = ""
    elif jType == 4:
        # not implemented
        jointType = ""
    elif jType == 5:
        # not implemented
        jointType = ""
    elif jType == 6:
        # SDFormat does not implement ball joint limits
        jointType = "ball"
    name = joi.name
    joint = ET.Element("joint", name=name, type=jointType)
    # build parent node
    parent = ET.Element("parent")
    parent.text = name_parent
    joint.append(parent)
    # build child node
    child = ET.Element("child")
    child.text = name_child
    joint.append(child)
    # build pose node
    pose = sdfPoseVector(joi.geometryOrOriginOne.origin)
    joint.append(pose)
    joint.extend(jointInfo)
    return joint

## Builds SDF axis node for revolute joints.
#
# This function builds the SDF axis node for revolute joint.
#
# @param joi one revolute joint object
# @return a list of information nodes (here one axis node)
# for the revolute joint
def revoluteJoint(joi):
    info = []
    # build axis node
    axis = ET.Element("axis")
    xyz = ET.Element("xyz")    
    vector = joi.jointMotion.rotationAxisVector
    xyz.text = vectorToString(vector.x, vector.y, vector.z)
    axis.append(xyz)
    # build limit node
    mini = joi.jointMotion.rotationLimits.minimumValue
    maxi = joi.jointMotion.rotationLimits.maximumValue
    limit = ET.Element("limit")
    axis.append(limit)
    # Lower and upper limit have to be switched and inverted,
    # because Fusion 360 moves the parent link wrt to the
    # child link and Gazebo moves the child link wrt to the
    # parent link.
    lower = ET.Element("lower")
    lower.text = str(-maxi)
    limit.append(lower)
    upper = ET.Element("upper")
    upper.text = str(-mini)
    limit.append(upper)
    # build frame node
    frame = ET.Element("use_parent_model_frame")
    frame.text = "0"
    axis.append(frame)
    info.append(axis)
    return info
    
## Plain STL export.
##
# @param occ the occurrence to be exported
# @param linkName the name of the created STL file
def exportToSTL(occ, linkName):
    global design
    global fileDir
    fileName = fileDir + "/meshes/" + linkName
    desExp = design.exportManager
    stlExportOptions = desExp.createSTLExportOptions(occ, fileName)
    desExp.execute(stlExportOptions)

## Exports a rigid group to STL.
## Transforms a matrix from Fusion 360 to Gazebo.
#
# This exports a rigid group as one STL file.
# For this all components of the rigidGroup are copied to a new component.
#
# @param rig the rigid group to be exported
# @return a new occurrence which is used to export the
# relevant information to SDFormat
def rigidGroupToSTL(rig):
    global rootOcc
    linkName = rig.name
    # create new occurrence
    linkOcc = rootOcc.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    # copy all bodies of the rigid group to the new occurrence
    allOcc = rig.occurrences
    for occ in allOcc:
        allBod = occ.bRepBodies
        for bod in allBod:
            bod.copyToComponent(linkOcc)
    # export new occurrence to STL
    exportToSTL(linkOcc, linkName)
    return linkOcc
    
## Exports an single occurrence to STL.
#
# This exports a single Fusion occurence as an STL file.
#
# @param occ the occurrence that needs to be exported.
# @return a new occurrence which is used to export the
# relevant information to SDFormat
def occurrenceToSTL(occ):
    global rootOcc
    linkName = clearName(occ.name)
    # create new occurrence
    linkOcc = rootOcc.addNewComponent(adsk.core.Matrix3D.create())
    linkOcc.component.name = linkName
    # copy all bodies of the occurrence to the new occurrence
    allBod = occ.bRepBodies
    for bod in allBod:
        bod.copyToComponent(linkOcc)
    # export new occurrence to STL
    exportToSTL(linkOcc, linkName)
    return linkOcc

## Clear filenames of unwanted characters
#
# This function replaces all ':' with underscores and deletes spaces in filenames.
# to one in the Gazebo cooridnate system.
#
# @param name a filename
# @return the filename without ':' and spaces
def clearName(name):
    name = name.replace(":", "_")
    name = name.replace(" ", "")
    return name

## A class to hold information about all muscles.
class Plugin:
    myoMuscles = []

## A class to hold information about all viaPoints of a muscle.
class MyoMuscle:
    number = ""
    viaPoints = []

## A class to hold information about a viaPoint.
class ViaPoint:
    coordinates = ""
    link = ""
    number = ""


## Exports a robot model from Fusion 360 to SDFormat.
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        # get active design        
        product = app.activeProduct
        global design
        design = adsk.fusion.Design.cast(product)
        # get root component in this design
        rootComp = design.rootComponent
        # get all occurrences within the root component
        global rootOcc
        rootOcc = rootComp.occurrences
        # build sdf root node
        root = ET.Element("sdf", version="1.6")
        model = ET.Element("model", name=modelName)
        root.append(model)
        # get all rigid groups of the root component
        allRigidGroups = rootComp.allRigidGroups
        # exports all rigid groups to STL and SDF
        for rig in allRigidGroups:
            if rig is not None:
                linkOcc = rigidGroupToSTL(rig)
                link = linkSDF(linkOcc)
                model.append(link)
                # delete the temporary new occurrence
                linkOcc.deleteMe()
                # Call doEvents to give Fusion a chance to react.
                adsk.doEvents()
        pluginObj = Plugin()
        #get all joints of the design
        allComponents = design.allComponents
        for com in allComponents:
            if com is not None:
                allJoints = com.joints
                for joi in allJoints:
                    if joi is not None:
                        one = joi.occurrenceOne
                        two = joi.occurrenceTwo
                        name_parent = clearName(one.name)
                        name_child = clearName(two.name)
                        missing_link = True
                        for rig in allRigidGroups:
                            value_parent = rig.occurrences.itemByName(one.name)
                            value_child = rig.occurrences.itemByName(two.name)
                            if value_parent is not None:
                                name_parent = rig.name
                            if value_child is not None:
                                name_child = rig.name
                                missing_link = False
                        joint = jointSDF(joi, name_parent, name_child)
                        model.append(joint)
                        # export missing links to SDF
                        if missing_link:
                            linkOcc = occurrenceToSTL(two)
                            link = linkSDF(linkOcc)
                            model.append(link)
                            # delete the temporary new occurrence
                            linkOcc.deleteMe()
                            # Call doEvents to give Fusion a chance to react.
                            adsk.doEvents()
                # get all construction points that serve as viaPoints
                allConstructionPoints = com.constructionPoints
                for point in allConstructionPoints:
                    if point is not None:
                        if point.name[:2] == "VP":
                            viaPointInfo = point.name.split("_")
                            viaPoint = ViaPoint()
                            p = point.geometry
                            viaPoint.coordinates = str(p.x) + " " + str(p.y) + " " + str(p.z)
                            viaPoint.link = viaPointInfo[2]
                            viaPoint.number = viaPointInfo[3]
                            myoNumber = viaPointInfo[1][5:]
                            myoMuscleList = list(filter(lambda x: x.number == myoNumber, pluginObj.myoMuscles))
                            if not myoMuscleList:
                                myoMuscle = MyoMuscle()
                                myoMuscle.viaPoints.append(viaPoint)
                                myoMuscle.number = myoNumber
                                pluginObj.myoMuscles.append(myoMuscle)
                            if myoMuscleList:
                                myoMuscleList[0].viaPoints.append(viaPoint) 
        global exportViaPoints
        if(exportViaPoints):
            # create plugin node
            global pluginFileName
            global pluginName
            plugin = ET.Element("plugin", filename=pluginFileName, name=pluginName)
            model.append(plugin)
            allMyoMuscles = pluginObj.myoMuscles
            allMyoMuscles.sort(key=lambda x: x.number)
            # create myoMuscle nodes
            for myo in allMyoMuscles:
                myoMuscle = ET.Element("myoMuscle", name="motor"+myo.number)
                plugin.append(myoMuscle)
                allViaPoints = myo.viaPoints
                allViaPoints.sort(key=lambda x: x.number)
                link = ET.Element("link", name="default")
                # create viaPoint nodes as children of links
                for via in allViaPoints:
                    if link.get("name") != via.link:
                        link = ET.Element("link", name=via.link)
                        myoMuscle.append(link)
                    # TODO: export more types of viaPoints
                    viaPoint = ET.Element("viaPoint", type="FIXPOINT")
                    # TODO: rotate global coordinates into link frame coordinates
                    viaPoint.text=via.coordinates
                    link.append(viaPoint)

        filename = fileDir + "/model.sdf"
        domxml = DOM.parseString(ET.tostring(root))
        pretty = domxml.toprettyxml()
        file = open(filename, "w")
        file.write(pretty)
        file.close()
        ui.messageBox("SDF file of model " + modelName + " written to '" + fileDir + "'.")
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))