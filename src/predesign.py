from abaqus import *
from abaqusConstants import *
# coding=utf-8
import __main__

import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
from abaqus import getWarningReply, YES, NO

def feasible_model(Long_Whole, Width_Whole, Mesh_Size_Whole, Long_Center, Width_Center,Mesh_Size_Center, Radius, Speed,
Mesh_Size_Impact,  Total_Time,
                   polymer_name, Metal_name, Stack, metal_num, polymer_num):
    model_name = 'Model-{}_{}'.format(metal_num, polymer_num)
    # create model
    mdb.Model(name=model_name, modelType=STANDARD_EXPLICIT)
    mdb.Model(name=model_name, objectToCopy=mdb.models['Model-1'])

    Speed = -Speed
    layers_num = len(Stack)


    thick_whole = 0
    for i in range(layers_num):
        thick_whole = thick_whole + Stack[i][2]

    create_plate(Long_Whole, Width_Whole, Long_Center, Width_Center, thick_whole, Stack, model_name)
    set_plate_property(Stack, Metal_name, polymer_name, model_name)
    set_assemble(Radius, thick_whole, model_name)
    set_step(Total_Time, Speed, model_name)
    set_interaction(model_name)
    set_load(model_name)
    set_mesh(Mesh_Size_Whole, Mesh_Size_Center, Mesh_Size_Impact, model_name)


def create_plate(Long_Whole, Width_Whole, Long_Center, Width_Center, thick_whole, Stack, model_name):
    # Part()
    # Create Plate
    # Sktech
    Long_Half = Long_Whole / 2.0
    Long_Quarter = Long_Half / 2.0
    Width_Half = Width_Whole / 2.0
    Width_Quarter = Width_Half / 2.0

    Long_Center_Half = Long_Center / 2.0
    Long_Center_Quarter = Long_Center / 4.0


    Width_Center_Half = Width_Center / 2.0
    Width_Center_Quarter = Width_Center / 4.0
    thick_half = thick_whole / 2.0

    s = mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    s.setPrimaryObject(option=STANDALONE)

    s.rectangle(point1=(0.0, 0.0), point2=(Long_Half, Width_Half))  ##--

    p = mdb.models[model_name].Part(name='Plate', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model_name].parts['Plate']
    # Extrusion
    p.BaseSolidExtrude(sketch=s, depth=thick_whole)  # --

    s.unsetPrimaryObject()
    del mdb.models[model_name].sketches['__profile__']


    # Partition
    # Layer Sketch

    f = p.faces.findAt((Long_Quarter, 0, thick_half))
    e = p.edges.findAt((Long_Half, 0, thick_half))
    t = p.MakeSketchTransform(sketchPlane=f, sketchUpEdge=e, sketchPlaneSide=SIDE1, origin=(0, 0, 0))

    s = mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=100, gridSpacing=5, transform=t)
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

    s.unsetPrimaryObject()
    del mdb.models[model_name].sketches['__profile__']




    # Focus Area Sketch
    f = p.faces.findAt((Long_Quarter, Width_Quarter, thick_whole))  ##--
    e = p.edges.findAt((Long_Half, Width_Quarter, thick_whole))  ##--
    t = p.MakeSketchTransform(sketchPlane=f, sketchUpEdge=e, sketchPlaneSide=SIDE1, origin=(0, 0, thick_whole))  # -

    s = mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=150, gridSpacing=3, transform=t)
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)

    s.Line(point1=(0.0, Width_Center_Half), point2=(Long_Center_Half, Width_Center_Half))  ##--
    s.Line(point1=(Long_Center_Half, Width_Center_Half), point2=(Long_Center_Half, 0))  ##--

    p.PartitionFaceBySketch(sketchUpEdge=e, faces=f, sketch=s)
    s.unsetPrimaryObject()
    del mdb.models[model_name].sketches['__profile__']

    p.ReferencePoint(point=(0, 0, thick_whole))  # --
    mdb.models[model_name].parts['Plate'].features.changeKey(fromName='RP', toName='Plate')

    # Focus Area Partition
    c = p.cells
    e = p.edges
    pickedEdges = (e.findAt((Long_Center_Quarter, Width_Center_Half, thick_whole)),
                   e.findAt((Long_Center_Half, Width_Center_Quarter, thick_whole)))  ##--
    tmp_num = float(Stack[0][2]) / 4.0
    p.PartitionCellByExtrudeEdge(line=e.findAt((0.0, 0.0, tmp_num)), cells=c, edges=pickedEdges,
                                 sense=REVERSE)  #######---



def create_impact_head(Radius, model_name):
    # Create Impact Head
    s = mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=30.0)
    g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ConstructionLine(point1=(0.0, -15.0), point2=(0.0, 15.0))

    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(Radius, 0.0), point2=(0.0, Radius), direction=COUNTERCLOCKWISE)  # -
    s.Line(point1=(0.0, Radius), point2=(0.0, 0.0))  # -
    s.Line(point1=(0.0, 0.0), point2=(Radius, 0.0))  # -

    p = mdb.models[model_name].Part(name='Impact Head', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model_name].parts['Impact Head']
    p.BaseSolidRevolve(sketch=s, angle=90.0, flipRevolveDirection=OFF)
    s.unsetPrimaryObject()
    del mdb.models[model_name].sketches['__profile__']

    p.ReferencePoint(point=(0.0, 0.0, Radius))  # -
    mdb.models[model_name].parts['Impact Head'].features.changeKey(fromName='RP', toName='Impact Head')
    r = p.referencePoints
    refPoints = (r[2],)
    p.Set(referencePoints=refPoints, name='Impact Head')


def create_support(model_name):

    # Create Support Up

    s = mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    s.setPrimaryObject(option=STANDALONE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(38.0, 0.0), point2=(0.0, 38.0), direction=COUNTERCLOCKWISE)

    s.Line(point1=(38.0, 0.0), point2=(60.0, 0.0))  # C
    s.Line(point1=(60.0, 0.0), point2=(60.0, 60.0))  # C
    s.Line(point1=(60.0, 60.0), point2=(0.0, 60.0))  # C
    s.Line(point1=(0.0, 60.0), point2=(0.0, 38.0))  # C

    p = mdb.models[model_name].Part(name='Support Up', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model_name].parts['Support Up']
    p.BaseSolidExtrude(sketch=s, depth=1.0)
    s.unsetPrimaryObject()
    del mdb.models[model_name].sketches['__profile__']

    p.ReferencePoint(point=(60.0, 30.0, 1.0))
    mdb.models[model_name].parts['Support Up'].features.changeKey(fromName='RP', toName='Support Up')
    r = p.referencePoints
    refPoints = (r[2],)
    p.Set(referencePoints=refPoints, name='Support Up')

    # Creative Support_Down

    s = mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    s.setPrimaryObject(option=STANDALONE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(38.0, 0.0), point2=(0.0, 38.0), direction=COUNTERCLOCKWISE)

    s.Line(point1=(38.0, 0.0), point2=(60.0, 0.0))
    s.Line(point1=(60.0, 0.0), point2=(60.0, 60.0))
    s.Line(point1=(60.0, 60.0), point2=(0.0, 60.0))
    s.Line(point1=(0.0, 60.0), point2=(0.0, 38.0))

    p = mdb.models[model_name].Part(name='Support Down', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model_name].parts['Support Down']
    p.BaseSolidExtrude(sketch=s, depth=1.0)
    s.unsetPrimaryObject()
    del mdb.models[model_name].sketches['__profile__']

    p.ReferencePoint(point=(60.0, 30.0, 0.0))
    mdb.models[model_name].parts['Support Down'].features.changeKey(fromName='RP', toName='Support Down')
    r = p.referencePoints
    refPoints = (r[2],)
    p.Set(referencePoints=refPoints, name='Support Down')


def set_assemble(Radius, thick_whole, model_name):
    print("in assemble")

    a = mdb.models[model_name].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models[model_name].parts['Impact Head']
    a.Instance(name='Impact Head', part=p, dependent=ON)
    p = mdb.models[model_name].parts['Plate']
    a.Instance(name='Plate', part=p, dependent=ON)
    p = mdb.models[model_name].parts['Support Down']
    a.Instance(name='Support Down', part=p, dependent=ON)
    p = mdb.models[model_name].parts['Support Up']
    a.Instance(name='Support Up', part=p, dependent=ON)

    a.translate(instanceList=('Support Up',), vector=(0.0, 0.0, thick_whole))  # -

    a.translate(instanceList=('Support Down',), vector=(0.0, 0.0, -1.0))

    a.rotate(instanceList=('Impact Head',), axisPoint=(0.0, 0.0, 0.0), axisDirection=(1.0, 0.0, 0.0), angle=-180.0)
    a.rotate(instanceList=('Impact Head',), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 1.0), angle=90.0)

    a.translate(instanceList=('Impact Head',), vector=(0.0, 0.0, Radius + thick_whole))  ##--

def set_property(model_name):
    print("in set_property")
    # Material Property
    # Rigid

    mdb.models[model_name].Material(name='Rigid')
    mdb.models[model_name].materials['Rigid'].Density(table=((1.019e-05, ), ))
    mdb.models[model_name].materials['Rigid'].Elastic(table=((1, 0.3), ))
    #

    # Section

    mdb.models[model_name].HomogeneousSolidSection(name='Rigid', material='Rigid', thickness=None)

    # Assign

    p = mdb.models[model_name].parts['Impact Head']
    cells = p.cells.getSequenceFromMask(mask=('[#1 ]',), )
    region = regionToolset.Region(cells=cells)
    p.SectionAssignment(region=region, sectionName='Rigid', offset=0.0,
                        offsetType=MIDDLE_SURFACE, offsetField='',
                        thicknessAssignment=FROM_SECTION)

    p = mdb.models[model_name].parts['Support Down']
    cells = p.cells.getSequenceFromMask(mask=('[#1 ]',), )
    region = regionToolset.Region(cells=cells)
    p = mdb.models[model_name].parts['Support Down']
    p.SectionAssignment(region=region, sectionName='Rigid', offset=0.0,
                        offsetType=MIDDLE_SURFACE, offsetField='',
                        thicknessAssignment=FROM_SECTION)

    p = mdb.models[model_name].parts['Support Up']
    cells = p.cells.getSequenceFromMask(mask=('[#1 ]',), )
    region = regionToolset.Region(cells=cells)
    # p = mdb.models['Model-1'].parts['Support Up']
    p.SectionAssignment(region=region, sectionName='Rigid', offset=0.0,
                        offsetType=MIDDLE_SURFACE, offsetField='',
                        thicknessAssignment=FROM_SECTION)



def set_plate_property(Stack, Metal_name, polymer_name, model_name):
    print("in set plate property")

    p = mdb.models[model_name].parts['Plate']
    c = p.cells

    layup = []
    num = 1
    for _ in Stack:
        if (_[0] != _[1]):
            if _[0]:
                layup.append(section.SectionLayer(material=Metal_name,
                                                  thickness=_[2], orientAngle=_[3], numIntPts=3, plyName="p%d" % num))
            else:
                layup.append(section.SectionLayer(material=polymer_name,
                                                  thickness=_[2], orientAngle=_[3], numIntPts=3, plyName="p%d" % num))
        num += 1

    mdb.models[model_name].CompositeShellSection(name='Section-1', preIntegrate=OFF, idealization=NO_IDEALIZATION,
                                                symmetric=False, thicknessType=UNIFORM, poissonDefinition=DEFAULT,
                                                thicknessModulus=None,
                                                temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON,
                                                layup=tuple(layup, ))
    p = mdb.models[model_name].parts['Plate']
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#3 ]',), )
    region = regionToolset.Region(cells=cells)
    p = mdb.models[model_name].parts['Plate']
    p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0,
                        offsetType=MIDDLE_SURFACE, offsetField='',
                        thicknessAssignment=FROM_SECTION)
    mdb.models[model_name].parts['Plate'].MaterialOrientation(region=region,
                                                              orientationType=SYSTEM, axis=AXIS_3,
                                                              localCsys=None,
                                                              fieldName='',
                                                              additionalRotationType=ROTATION_ANGLE,
                                                              additionalRotationField='', angle=0.0,
                                                              stackDirection=STACK_3)



def set_step(Total_Time, Speed, model_name):
    print("in step")
    # Step()
    mdb.models[model_name].ExplicitDynamicsStep(name='Step-1', previous='Initial', timePeriod=Total_Time)  # -

    a = mdb.models[model_name].rootAssembly
    r1 = a.instances['Impact Head'].referencePoints
    refPoints1 = (r1[2],)
    region = regionToolset.Region(referencePoints=refPoints1)
    mdb.models[model_name].Velocity(name='Predefined Field-1', region=region,
                                   field='', distributionType=MAGNITUDE, velocity1=0.0, velocity2=0.0,
                                   velocity3=Speed, omega=0.0)  # -

    mdb.models[model_name].FieldOutputRequest(name='F-Output-2',
                                             createStepName='Step-1', variables=(
        'SDEG', 'DMICRT', 'CSDMG', 'CSQUADSCRT', 'SDV', 'FV', 'MFR', 'UVARM',
        'EMSF', 'DENSITY', 'DENSITYVAVG', 'STATUS', 'RHOE', 'RHOP', 'BURNF',
        'DBURNF', 'TIEDSTATUS', 'TIEADJUST'))

    regionDef = mdb.models[model_name].rootAssembly.instances['Impact Head'].sets['Impact Head']
    mdb.models[model_name].HistoryOutputRequest(name='H-Output-2',
                                               createStepName='Step-1', variables=('S11', 'S22', 'S33', 'S12', 'S13',
                                                                                   'S23', 'SP', 'TRESC', 'PRESS',
                                                                                   'INV3', 'MISES', 'TSHR13', 'TSHR23',
                                                                                   'CTSHR13', 'CTSHR23', 'TRIAX',
                                                                                   'VS11', 'VS22', 'VS33', 'VS12',
                                                                                   'VS13',
                                                                                   'VS23', 'PS11', 'PS22', 'PS33',
                                                                                   'PS12', 'PS13', 'PS23', 'SFABRIC11',
                                                                                   'SFABRIC22', 'SFABRIC33',
                                                                                   'SFABRIC12', 'SFABRIC13',
                                                                                   'SFABRIC23',
                                                                                   'SSAVG1', 'SSAVG2', 'SSAVG3',
                                                                                   'SSAVG4', 'SSAVG5', 'SSAVG6', 'U1',
                                                                                   'U2',
                                                                                   'U3', 'UR1', 'UR2', 'UR3', 'UT',
                                                                                   'UR', 'UCOM1', 'UCOM2', 'UCOM3',
                                                                                   'V1',
                                                                                   'V2', 'V3', 'VR1', 'VR2', 'VR3',
                                                                                   'VT', 'VR', 'VCOM1', 'VCOM2',
                                                                                   'VCOM3',
                                                                                   'A1', 'A2', 'A3', 'AR1', 'AR2',
                                                                                   'AR3', 'AT', 'AR', 'ACOM1', 'ACOM2',
                                                                                   'ACOM3', 'RBANG', 'RBROT', 'SDV',
                                                                                   'FV', 'MFR', 'UVARM', 'DT', 'DMASS',
                                                                                   'EMSF', 'DENSITY', 'STATUS', 'RHOE',
                                                                                   'RHOP', 'BURNF', 'DBURNF'),
                                               region=regionDef, sectionPoints=DEFAULT, rebar=EXCLUDE)

def  set_interaction(model_name):
    a = mdb.models[model_name].rootAssembly
    p = mdb.models[model_name].parts['Impact Head']
    p = mdb.models[model_name].parts['Plate']
    a = mdb.models[model_name].rootAssembly
    p1 = mdb.models[model_name].parts['Support Down']
    a = mdb.models[model_name].rootAssembly
    mdb.models[model_name].ContactProperty('IntProp-1')
    mdb.models[model_name].interactionProperties['IntProp-1'].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON,
        constraintEnforcementMethod=DEFAULT)
    mdb.models[model_name].interactionProperties['IntProp-1'].TangentialBehavior(
        formulation=FRICTIONLESS)
    #: The interaction property "IntProp-1" has been created.

    mdb.models[model_name].ContactExp(name='Int-1', createStepName='Initial')
    mdb.models[model_name].interactions['Int-1'].includedPairs.setValuesInStep(
        stepName='Initial', useAllstar=ON)
    mdb.models[model_name].interactions['Int-1'].contactPropertyAssignments.appendInStep(
        stepName='Initial', assignments=((GLOBAL, SELF, 'IntProp-1'),))

    #: The interaction "Int-1" has been created.


    a = mdb.models[model_name].rootAssembly
    c1 = a.instances['Impact Head'].cells
    cells1 = c1.getSequenceFromMask(mask=('[#1 ]',), )
    region2 = regionToolset.Region(cells=cells1)
    a = mdb.models[model_name].rootAssembly
    r1 = a.instances['Impact Head'].referencePoints
    refPoints1 = (r1[2],)
    region1 = regionToolset.Region(referencePoints=refPoints1)
    mdb.models[model_name].RigidBody(name='Constraint-1', refPointRegion=region1,
                                    bodyRegion=region2)
    a = mdb.models[model_name].rootAssembly
    c1 = a.instances['Support Up'].cells
    cells1 = c1.getSequenceFromMask(mask=('[#1 ]',), )
    region2 = regionToolset.Region(cells=cells1)
    a = mdb.models[model_name].rootAssembly
    r1 = a.instances['Support Up'].referencePoints
    refPoints1 = (r1[2],)
    region1 = regionToolset.Region(referencePoints=refPoints1)
    mdb.models[model_name].RigidBody(name='Constraint-2', refPointRegion=region1,
                                    bodyRegion=region2)
    a = mdb.models[model_name].rootAssembly
    c1 = a.instances['Support Down'].cells
    cells1 = c1.getSequenceFromMask(mask=('[#1 ]',), )
    region2 = regionToolset.Region(cells=cells1)
    a = mdb.models[model_name].rootAssembly
    r1 = a.instances['Support Down'].referencePoints
    refPoints1 = (r1[2],)
    region1 = regionToolset.Region(referencePoints=refPoints1)
    mdb.models[model_name].RigidBody(name='Constraint-3', refPointRegion=region1,
                                    bodyRegion=region2)

def set_load(model_name):
    a = mdb.models[model_name].rootAssembly
    f1 = a.instances['Support Down'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#40 ]',), )
    f2 = a.instances['Support Up'].faces
    faces2 = f2.getSequenceFromMask(mask=('[#20 ]',), )
    region = regionToolset.Region(faces=faces1 + faces2)
    mdb.models[model_name].EncastreBC(name='BC-1', createStepName='Initial',
                                     region=region, localCsys=None)
    a = mdb.models[model_name].rootAssembly
    r1 = a.instances['Impact Head'].referencePoints
    refPoints1 = (r1[2],)
    region = regionToolset.Region(referencePoints=refPoints1)
    mdb.models[model_name].XsymmBC(name='imp-x', createStepName='Initial',
                                  region=region, localCsys=None)
    a = mdb.models[model_name].rootAssembly
    r1 = a.instances['Impact Head'].referencePoints
    refPoints1 = (r1[2],)
    region = regionToolset.Region(referencePoints=refPoints1)
    mdb.models[model_name].YsymmBC(name='imp-y', createStepName='Initial',
                                  region=region, localCsys=None)
    a = mdb.models[model_name].rootAssembly
    f1 = a.instances['Plate'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#180 ]',), )
    region = regionToolset.Region(faces=faces1)
    mdb.models[model_name].EncastreBC(name='plate', createStepName='Initial',
                                     region=region, localCsys=None)
    a = mdb.models[model_name].rootAssembly
    f1 = a.instances['Plate'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#50 ]',), )
    region = regionToolset.Region(faces=faces1)
    mdb.models[model_name].XsymmBC(name='sy-x', createStepName='Initial',
                                  region=region, localCsys=None)
    a = mdb.models[model_name].rootAssembly
    f1 = a.instances['Plate'].faces
    faces1 = f1.getSequenceFromMask(mask=('[#208 ]',), )
    region = regionToolset.Region(faces=faces1)
    mdb.models[model_name].YsymmBC(name='sy-y', createStepName='Initial',
                                  region=region, localCsys=None)


def set_mesh(Mesh_Size_Whole, Mesh_Size_Center, Mesh_Size_Impact, model_name):
    p = mdb.models[model_name].parts['Plate']
    c = p.cells
    pickedRegions = c.getSequenceFromMask(mask=('[#3 ]',), )
    p.setMeshControls(regions=pickedRegions, technique=SWEEP)
    p = mdb.models[model_name].parts['Plate']
    c, e = p.cells, p.edges
    p.setSweepPath(region=c[1], edge=e[6], sense=REVERSE)
    elemType1 = mesh.ElemType(elemCode=SC8R, elemLibrary=EXPLICIT,
                              secondOrderAccuracy=OFF, hourglassControl=DEFAULT, elemDeletion=ON)
    elemType2 = mesh.ElemType(elemCode=SC6R, elemLibrary=EXPLICIT)
    elemType3 = mesh.ElemType(elemCode=UNKNOWN_TET, elemLibrary=EXPLICIT)
    p = mdb.models[model_name].parts['Plate']
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#3 ]',), )
    pickedRegions = (cells,)
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2,
                                                       elemType3))
    p = mdb.models[model_name].parts['Plate']
    e = p.edges
    pickedEdges = e.getSequenceFromMask(mask=('[#182835 ]',), )
    p.seedEdgeBySize(edges=pickedEdges, size=Mesh_Size_Center, deviationFactor=0.1,
                     constraint=FINER)
    p = mdb.models[model_name].parts['Plate']
    e = p.edges
    pickedEdges = e.getSequenceFromMask(mask=('[#609042 ]',), )
    p.seedEdgeByNumber(edges=pickedEdges, number=1, constraint=FINER)
    p = mdb.models[model_name].parts['Plate']
    p.seedPart(size=Mesh_Size_Whole, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models[model_name].parts['Plate']
    p.generateMesh()

    #:
    #: Part: Plate
    #:   Number of elements :  7407,   Analysis errors:  0 (0%),  Analysis warnings:  0 (0%)
    p = mdb.models[model_name].parts['Impact Head']
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    elemType1 = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT,
                              kinematicSplit=AVERAGE_STRAIN, secondOrderAccuracy=OFF,
                              hourglassControl=DEFAULT, distortionControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=EXPLICIT)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=EXPLICIT)
    p = mdb.models[model_name].parts['Impact Head']
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#1 ]',), )
    pickedRegions = (cells,)
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2,
                                                       elemType3))
    p = mdb.models[model_name].parts['Impact Head']
    p.seedPart(size=Mesh_Size_Impact, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models[model_name].parts['Impact Head']
    p.generateMesh()
    p = mdb.models[model_name].parts['Support Down']
    elemType1 = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT,
                              kinematicSplit=AVERAGE_STRAIN, secondOrderAccuracy=OFF,
                              hourglassControl=DEFAULT, distortionControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=EXPLICIT)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=EXPLICIT)
    p = mdb.models[model_name].parts['Support Down']
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#1 ]',), )
    pickedRegions = (cells,)
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2,
                                                       elemType3))
    p = mdb.models[model_name].parts['Support Down']
    p.seedPart(size=3.0, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models[model_name].parts['Support Down']
    p.generateMesh()

    p = mdb.models[model_name].parts['Support Up']
    p.seedPart(size=3.0, deviationFactor=0.1, minSizeFactor=0.1)
    p = mdb.models[model_name].parts['Support Up']
    p.generateMesh()
    elemType1 = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT,
                              kinematicSplit=AVERAGE_STRAIN, secondOrderAccuracy=OFF,
                              hourglassControl=DEFAULT, distortionControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=EXPLICIT)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=EXPLICIT)
    p = mdb.models[model_name].parts['Support Up']
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#1 ]',), )
    pickedRegions = (cells,)
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2,
                                                       elemType3))
    p = mdb.models[model_name].parts['Plate']
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    a1 = mdb.models[model_name].rootAssembly
    a1.regenerate()
    a = mdb.models[model_name].rootAssembly






def Test(targetThickness, max_density, min_tensile_modulus, metalCombobox, m_min_thick, m_max_thick,  polymerCombobox,
         p_min_thick, p_max_thick,
Long_Whole, Width_Whole, Mesh_Size_Whole, Long_Center, Width_Center,Mesh_Size_Center, Radius, Speed,
Mesh_Size_Impact,  Total_Time):

    mModulus = mdb.models['Model-1'].materials[metalCombobox].elastic.table[0][0]
    pModulus = mdb.models['Model-1'].materials[polymerCombobox].elastic.table[0][0]
    mDensity = mdb.models['Model-1'].materials[metalCombobox].density.table[0][0] * 1e12
    pDensity = mdb.models['Model-1'].materials[polymerCombobox].density.table[0][0] * 1e12
    maxVm, res_lst = design(targetThickness, max_density, min_tensile_modulus, metalCombobox, m_min_thick, m_max_thick, polymerCombobox,
           p_min_thick, p_max_thick, mModulus, pModulus, mDensity, pDensity)

    # Model-1  setting

    create_impact_head(Radius, "Model-1")
    create_support("Model-1")
    set_property("Model-1")

    # fangan_lst is not empty

    if res_lst:
        polymer_name = polymerCombobox
        Metal_name = metalCombobox
        for i, fangan in enumerate(res_lst):
            Stack = []
            for i in range(fangan[0] + fangan[1]):
                if i % 2 == 0:   # Metal
                    Stack.append((True, False, fangan[2], 0))
                else:      # polymer
                    Stack.append((False, True, fangan[3], 0))

            feasible_model(Long_Whole, Width_Whole, Mesh_Size_Whole, Long_Center, Width_Center, Mesh_Size_Center, Radius, Speed,
                           Mesh_Size_Impact, Total_Time,
                           polymer_name, Metal_name, Stack, fangan[0], fangan[1])
        print("\n\n")
        print("################################################################################################################################")
        print("Number        Structure          Metal(mm)          Polymer(mm)            FML density(kg/m3)           FML modulus(MPa)")
        print("################################################################################################################################")

        fmlDensity = "%.2f" %(maxVm * mDensity + (1 - maxVm) * pDensity)
        fmlModulus = "%.2f" %(maxVm * mModulus + (1 - maxVm) * pModulus)
        for i, fangan in enumerate(res_lst):
            print("  {}             {}/{}               {}                 {}                      {}                     {}".
                  format(i + 1, fangan[0], fangan[1], "%.2f" %fangan[2], "%.2f" %fangan[3],
                         fmlDensity, fmlModulus))

    else:
        reply = getWarningReply('There is no feasible design for the material system.\nOkay to continue?', (YES, NO))
        if reply == NO:
            print(1/0)
        else:
            pass

def design(targetThickness, max_density, min_tensile_modulus, metalCombobox, m_min_thick, m_max_thick, polymerCombobox,
           p_min_thick, p_max_thick, mModulus, pModulus, mDensity, pDensity):

    max_Vm = calculateMaxVm(max_density, min_tensile_modulus, mModulus, pModulus, mDensity, pDensity)
    if max_Vm == -1:
        return [], []
    else:
        metal_total_thickness = targetThickness * max_Vm
        polymer_total_thickness = targetThickness - metal_total_thickness

        m_max_num = int(metal_total_thickness / m_min_thick) + 1
        p_max_num = int(polymer_total_thickness / p_min_thick) + 1
        m_min_num = int(metal_total_thickness / m_max_thick)
        p_min_num = int(polymer_total_thickness / p_max_thick)
        fangan_lst = []
        for mNum in range(max(m_min_num, p_min_num, 2), min(m_max_num, p_max_num) + 2):
            pNum = mNum - 1
            metal_each_layer_thickness = metal_total_thickness / mNum
            polymer_each_layer_thickness = polymer_total_thickness / pNum
            if (m_min_thick < metal_each_layer_thickness < p_max_thick) and (
                    p_min_thick < polymer_each_layer_thickness < p_max_thick):
                fangan_lst.append((mNum, pNum, metal_each_layer_thickness,
                                   polymer_each_layer_thickness))
        return max_Vm, fangan_lst

def calculateMaxVm(max_density, min_tensile_modulus, mModulus, pModulus, mDensity, pDensity):
    # calculate density, modulus constraints , Vm + Vf = 1

    density_point = (max_density - pDensity) / (mDensity - pDensity)
    modulus_point = (min_tensile_modulus - pModulus) / (mModulus - pModulus)

    if mDensity <= pDensity and mModulus >= pModulus:

        # metal can statisfy the constraints

        return 1

    elif mDensity <= pDensity and mModulus < pModulus:

        # Vm  > density_point Vm < modulus_point

        if density_point > 1 or modulus_point < 0 or modulus_point < density_point:
            return -1
        else:
            return min(modulus_point, 1)

    elif mDensity > pDensity and mModulus >= pModulus:

        # Vm  < density_point Vm > modulus_point

        if density_point < 0 or modulus_point > 1 or modulus_point > density_point:
            return -1
        else:
            return min(density_point, 1)

    else:

        # Vm  < density_point Vm < modulus_point

        if density_point < 0 or modulus_point < 0:
            return -1
        else:
            return min(density_point, modulus_point)



