import adsk.core
import adsk.fusion
import traceback
import xml.etree.ElementTree as ET
import math
import xml.dom.minidom as DOM

# Global variables
design = None
fileDir = ""
modelName = ""
rootOcc = None

# Transforms Fusion 360 coordinate system to Gazebo cooridnate system.
def gazeboMatrix(m):
    matrix = adsk.core.Matrix3D.create()
    matrix.setCell(1, 1, 0)
    matrix.setCell(1, 2, -1)
    matrix.setCell(2, 1, 1)
    matrix.setCell(2, 2, 0)
    m.transformBy(matrix)
    return m

# Converts three double values to string.
def vectorToString(x, y, z):
    string = str(x) + " " + str(y) + " " + str(z)
    return string

# Builds SDF pose node from vector.
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
    
# Builds SDF pose node from matrix.
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

# Builds SDF inertial node from physical properties.
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

# Builds SDF node for one moment of inertia.
def sdfMom(tag, value):
    node = ET.Element(tag)
    # convert from kg/cm^2 (Fusion 360) to kg/m^2 (SI)
    node.text = str(0.0001 * value)
    return node

# Builds SDF inertia node from physical properties.
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
    
# Builds SDF link node.
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
    
# Builds SDF joint node.
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

# Builds SDF axis node for revolute joints.
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
    
# Plain STL export.
def exportToSTL(occ, linkName):
    global design
    global fileDir
    fileName = fileDir + "/meshes/" + linkName
    desExp = design.exportManager
    stlExportOptions = desExp.createSTLExportOptions(occ, fileName)
    desExp.execute(stlExportOptions)

# Exports a rigid group to STL.
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
    
# Export an single occurrence to STL.
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

# Clear filenames of unwanted characters
def clearName(name):
    name = name.replace(":", "_")
    name = name.replace(" ", "")
    return name
        
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
        # specify output location
        global fileDir
        fileDir = "C:/Users/techtalentsVR1/Documents/roboy/fusion/SDFusion"
        # specify model name
        global modelName
        modelName = "Roboy"
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