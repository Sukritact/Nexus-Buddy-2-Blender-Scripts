bl_info = {
	"name": "Export CivNexus6 (.cn6)",
	"author": "Deliverator",
	"version": (1, 0),
	"blender": (2, 80, 0),
	"location": "File > Export > CivNexus6 (.cn6)",
	"description": "Export CivNexus6 (.cn6)",
	"warning": "",
	"wiki_url": "",
	"category": "Import-Export"}

import bpy
import bmesh
from mathutils import Vector, Quaternion, Matrix
from bpy_extras.io_utils import unpack_list, unpack_face_list, ExportHelper
import math
import array
from bpy.props import (
		BoolProperty,
		FloatProperty,
		StringProperty,
		EnumProperty,
		)

BlenderVersion = bpy.app.version

def getTranslationOrientation(ob):
	if isinstance(ob, bpy.types.Bone):

		ob_matrix_local = ob.matrix_local.copy()
		ob_matrix_local.transpose()
		t = ob_matrix_local
		ob_matrix_local = Matrix([[-t[2][0], -t[2][1], -t[2][2], -t[2][3]],
								[t[1][0], t[1][1], t[1][2], t[1][3]],
								[t[0][0], t[0][1], t[0][2], t[0][3]],
								[t[3][0], t[3][1], t[3][2], t[3][3]]])

		rotMatrix_z90_4x4 = Matrix.Rotation(math.radians(90.0), 4, 'Z')
		rotMatrix_z90_4x4.transpose()

		t = rotMatrix_z90_4x4 @ ob_matrix_local
		matrix = Matrix([[t[0][0], t[0][1], t[0][2], t[0][3]],
								[t[1][0], t[1][1], t[1][2], t[1][3]],
								[t[2][0], t[2][1], t[2][2], t[2][3]],
								[t[3][0], t[3][1], t[3][2], t[3][3]]])

		parent = ob.parent
		if parent:
			parent_matrix_local = parent.matrix_local.copy()
			parent_matrix_local.transpose()
			t = parent_matrix_local
			parent_matrix_local = Matrix([[-t[2][0], -t[2][1], -t[2][2], -t[2][3]],
									[t[1][0], t[1][1], t[1][2], t[1][3]],
									[t[0][0], t[0][1], t[0][2], t[0][3]],
									[t[3][0], t[3][1], t[3][2], t[3][3]]])
			par_matrix = rotMatrix_z90_4x4 @ parent_matrix_local
			par_matrix_cpy = par_matrix.copy()
			par_matrix_cpy.invert()
			matrix = matrix @ par_matrix_cpy

		matrix.transpose()
		loc, rot, sca = matrix.decompose()
	else:
		matrix = ob.matrix_world
		if matrix:
			loc, rot, sca = matrix.decompose()
		else:
			raise "error: this should never happen!"
	return loc, rot

def getBoneTreeDepth(bone, currentCount):
	if (bone.parent):
		currentCount = currentCount + 1
		return getBoneTreeDepth(bone.parent, currentCount)
	else:
		return currentCount


def BPyMesh_meshWeight2List(ob, me):
	""" Takes a mesh and return its group names and a list of lists, one list per vertex.
	aligning the each vert list with the group names, each list contains float value for the weight.
	These 2 lists can be modified and then used with list2MeshWeight to apply the changes.
	"""

	# Clear the vert group.
	groupNames = [g.name for g in ob.vertex_groups]
	len_groupNames = len(groupNames)

	if not len_groupNames:
		# no verts? return a vert aligned empty list
		return [[] for i in range(len(me.vertices))], []
	else:
		vWeightList = [[0.0] * len_groupNames for i in range(len(me.vertices))]

	for i, v in enumerate(me.vertices):
		for g in v.groups:
			# possible weights are out of range
			index = g.group
			if index < len_groupNames:
				vWeightList[i][index] = g.weight

	return groupNames, vWeightList


def meshNormalizedWeights(ob, me):
	groupNames, vWeightList = BPyMesh_meshWeight2List(ob, me)

	if not groupNames:
		return [], []

	for i, vWeights in enumerate(vWeightList):
		tot = 0.0
		for w in vWeights:
			tot += w

		if tot:
			for j, w in enumerate(vWeights):
				vWeights[j] = w / tot

	return groupNames, vWeightList

def getBoneWeights(boneName, weights):
	if boneName in weights[0]:
		group_index = weights[0].index(boneName)
		vgroup_data = [(j, weight[group_index]) for j, weight in enumerate(weights[1]) if weight[group_index]]
	else:
		vgroup_data = []

	return vgroup_data

def do_export(context, filename, triangulate, use_selection):
	print ("Start CN6 Export...")

	file = open( filename, 'w')
	filedata = "// CivNexus6 CN6 - Exported from Blender for import to CivNexus6\n"

	try:
		modelObs = {}
		modelMeshes = {}
		depsgraph = context.evaluated_depsgraph_get()

		objectSet = bpy.data.objects
		if use_selection:
			objectSet = bpy.context.selected_objects

		for object in objectSet:

			if object.type == 'ARMATURE':
				modelObs[object.name] = object

			if object.type == 'MESH':
				print ("Getting parent for mesh: %s" % object.name)

				for modifier in object.modifiers:
					if modifier.type == "ARMATURE" and modifier.object is not None and modifier.object.type == "ARMATURE":
						parentArmOb = modifier.object
						if not parentArmOb.name in modelMeshes:
							modelMeshes[parentArmOb.name] = []
						modelMeshes[parentArmOb.name].append(object.evaluated_get(depsgraph))

						break

		for modelObName in modelObs.keys():
			boneIds = {}

			# Write Skeleton
			filedata += "skeleton\n"

			armOb = modelObs[modelObName]
			armature = armOb.data

			# Calc bone depths and sort
			boneDepths = []
			for bone in armature.bones.values():
				boneDepth = getBoneTreeDepth(bone, 0)
				boneDepths.append((bone, boneDepth))

			boneDepths = sorted(boneDepths, key=lambda k: k[0].name)
			boneDepths = sorted(boneDepths, key=lambda k: k[1])
			sortedBones = boneDepths

			for boneid, boneTuple in enumerate(sortedBones):
				boneIds[boneTuple[0].name] = boneid

			boneIds[armOb.name] = -1 # Add entry for World Bone

			# Write World Bone
			filedata += '%d "%s" %d ' % (0, armOb.name, -1)
			filedata += '%.8f %.8f %.8f ' % (0.0, 0.0, 0.0)
			filedata += '%.8f %.8f %.8f %.8f ' % (0.0, 0.0, 0.0, 1.0)
			filedata += '%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f\n' % (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)

			print ("armOb.name/armature.bones[0].name/len(boneIds)")
			print (armOb.name)
			print (armature.bones[0].name)
			print (len(boneIds))

			if (len(boneIds) > 1 or armOb.name != armature.bones[0].name):
				for boneid, boneTuple in enumerate(sortedBones):
					bone = boneTuple[0]
					#boneDepth = boneTuple[1]

					position, orientationQuat = getTranslationOrientation(bone)

					# Get Inverse World Matrix for bone
					x = bone.matrix_local.copy()
					x.transpose()
					t = Matrix([[-x[2][0], -x[2][1], -x[2][2], -x[2][3]],
								[x[1][0], x[1][1], x[1][2], x[1][3]],
								[x[0][0], x[0][1], x[0][2], x[0][3]],
								[x[3][0], x[3][1], x[3][2], x[3][3]]])
					t.invert()
					invWorldMatrix = Matrix([[t[0][1], -t[0][0], t[0][2], t[0][3]],
										[t[1][1], -t[1][0], t[1][2], t[1][3]],
										[t[2][1], -t[2][0], t[2][2], t[2][3]],
										[t[3][1], -t[3][0], t[3][2], t[3][3]]])

					outputBoneName = bone.name

					filedata += '%d "%s" ' % (boneid + 1, outputBoneName)   # Adjust bone ids + 1 as zero is the World Bone

					parentBoneId = 0
					if bone.parent:
						parentBoneId = boneIds[bone.parent.name] + 1   # Adjust bone ids + 1 as zero is the World Bone

					filedata += '%d ' % parentBoneId
					filedata +='%.8f %.8f %.8f ' % (position[0], position[1], position[2])
					filedata +='%.8f %.8f %.8f %.8f ' % (orientationQuat[1], orientationQuat[2], orientationQuat[3], orientationQuat[0]) # GR2 uses x,y,z,w for Quaternions rather than w,x,y,z
					filedata += '%.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f %.8f' % (invWorldMatrix[0][0], invWorldMatrix[0][1], invWorldMatrix[0][2], invWorldMatrix[0][3],
																						invWorldMatrix[1][0], invWorldMatrix[1][1], invWorldMatrix[1][2], invWorldMatrix[1][3],
																						invWorldMatrix[2][0], invWorldMatrix[2][1], invWorldMatrix[2][2], invWorldMatrix[2][3],
																						invWorldMatrix[3][0], invWorldMatrix[3][1], invWorldMatrix[3][2], invWorldMatrix[3][3])
					#End of bone line
					filedata += "\n"

			if len(modelMeshes) == 0:
				filedata += 'meshes:%d\n' % 0
			else:
				filedata += 'meshes:%d\n' % len(modelMeshes[modelObName])

				for meshObject in modelMeshes[modelObName]:

					dup_meshObject = meshObject.copy
					mesh = meshObject.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
					mesh = mesh.copy()

					bm = bmesh.new()
					bm.from_mesh(mesh)

					bmesh_edges = []
					for e in bm.edges:
						if not e.smooth:
							bmesh_edges.append(e)
					bmesh.ops.split_edges(bm, edges=bmesh_edges)

					if triangulate:
						bmesh.ops.triangulate(bm, faces=bm.faces[:])

					# Finish up, write the bmesh back to the mesh
					bm.to_mesh(mesh)
					bm.free()

					meshName = meshObject.name

					filedata += 'mesh:"%s"\n' % meshName

					filedata += 'materials\n'
					for material in meshObject.data.materials:
						filedata += '\"%s\"\n' % material.name

					# Read in preserved Normals, Binormals and Tangents
					FaceCornerBinormalsTangents = {}

					# This will wipe out custom normals
					mesh.update()
					if BlenderVersion < (4,1,0):
						mesh.calc_normals_split()
					mesh.calc_tangents(uvmap = mesh.uv_layers[0].name)

					for face in mesh.polygons:
						faceId = face.index
						FaceCornerBinormalsTangents[faceId] = {}

						for vert in [mesh.loops[i] for i in face.loop_indices]:
							vertexId = vert.vertex_index
							normal = vert.normal

							tangent = vert.tangent
							normal = vert.normal
							bitangent = vert.bitangent_sign * normal.cross(tangent)

							FaceCornerBinormalsTangents[faceId][vertexId] = (
								normal[0], normal[1], normal[2],
								tangent[0], tangent[1], tangent[2],
								bitangent[0], bitangent[1], bitangent[2]
							)

					# Get Bone Weights
					weights = meshNormalizedWeights(meshObject, mesh)
					vertexBoneWeights = {}
					print (meshName)
					print ("len(mesh.polygons)")
					print (len(mesh.polygons))
					print ("len(mesh.loops)")
					print (len(mesh.loops))
					print ("len(weights[0])")
					print (len(weights[0]))

					for boneName in boneIds.keys():
						vgroupDataForBone = getBoneWeights(boneName, weights)

						for vgData in vgroupDataForBone:
							vertexId = vgData[0]
							weight = vgData[1]
							if not vertexId in vertexBoneWeights:
								vertexBoneWeights[vertexId] = []
							vertexBoneWeights[vertexId].append((boneName, weight))

					print ("len(mesh.vertices)")
					print (len(mesh.vertices))
					print ("len(vertexBoneWeights.keys())")
					print (len(vertexBoneWeights.keys()))

					grannyVertexBoneWeights = {}
					for vertId in vertexBoneWeights.keys():

						rawBoneIdWeightTuples = []
						firstBoneId = 0
						for i in range(max(8,len(vertexBoneWeights[vertId]))):
							if i < len(vertexBoneWeights[vertId]):
								vertexBoneWeightTuple = vertexBoneWeights[vertId][i]
								boneName = vertexBoneWeightTuple[0]
								rawBoneIdWeightTuples.append((boneIds[boneName] + 1, vertexBoneWeightTuple[1]))
								if i == 0:
									firstBoneId = boneIds[boneName] + 1
							else:
								rawBoneIdWeightTuples.append((firstBoneId, 0))

						# Sort bone mappings by weight highest to lowest
						sortedBoneIdWeightTuples = sorted(rawBoneIdWeightTuples, key=lambda rawBoneIdWeightTuple: rawBoneIdWeightTuple[1], reverse=True)

						# Pick first 8 highest weighted bones
						boneIdsList = []
						rawBoneWeightsList = []
						for i in range(8):
							boneIdsList.append(sortedBoneIdWeightTuples[i][0])
							rawBoneWeightsList.append(sortedBoneIdWeightTuples[i][1])

						rawWeightTotal = 0
						for weight in rawBoneWeightsList:
							rawWeightTotal = rawWeightTotal + weight

						boneWeightsList = []
						for weight in rawBoneWeightsList:
							calcWeight = round(255 * weight / rawWeightTotal)
							boneWeightsList.append(calcWeight)

						# Ensure that total of vertex bone weights is 255
						runningTotal = 0
						for i, weight in enumerate(boneWeightsList):
							runningTotal = runningTotal + weight

						if runningTotal != 255:
							boneWeightsList[0] = boneWeightsList[0] + (255 - runningTotal)

						runningTotal = 0
						for i, weight in enumerate(boneWeightsList):
							runningTotal = runningTotal + weight

						#print("Current Running Total")
						#print(runningTotal)

						if runningTotal != 255:
							raise "Error: Vertex bone weights do not total 255!"

						if not vertId in grannyVertexBoneWeights:
							grannyVertexBoneWeights[vertId] = []
						grannyVertexBoneWeights[vertId] = (boneIdsList, boneWeightsList)

					position, orientationQuat = getTranslationOrientation(meshObject)

					filedata += "vertices\n"

					print ("grannyVertexBoneWeights")
					print (len(grannyVertexBoneWeights))
					print ("Write Vertices")

					# Get unique vertex/uv coordinate combinations
					uniqueVertSet = set()
					uniqueVertUVIndexes = {}
					uniqueVertUVs = []
					currentVertUVIndex = 0

					currentTriangleId = 0
					triangleVertUVIndexes = []
					triangleMaterialIndexes = []

					for poly in mesh.polygons:
						triangleVertUVIndexes.append([])

						faceId = poly.index

						for loop_index in poly.loop_indices:
							vertexId = mesh.loops[loop_index].vertex_index

							if (mesh.uv_layers[0]):
								uv = mesh.uv_layers[0].data[loop_index].uv
							else:
								uv = (0.0, 1.0)

							if (len(mesh.uv_layers) > 1):
								uv2 = mesh.uv_layers[1].data[loop_index].uv
							else:
								uv2 = (0.0, 1.0)

							if (len(mesh.uv_layers) > 2):
								uv3 = mesh.uv_layers[2].data[loop_index].uv
							else:
								uv3 = (0.0, 1.0)

							uvt = tuple(uv)
							uv2t = tuple(uv2)
							uv3t = tuple(uv3)

							vertSig = '%i|%.8f|%.8f|%.8f|%.8f|%.8f|%.8f' % (vertexId, uvt[0], uvt[1], uv2t[0], uv2t[1], uv3t[0], uv3t[1])

							if vertSig in uniqueVertSet:
								triangleVertUVIndex = uniqueVertUVIndexes[vertSig]
							else:
								uniqueVertSet.add(vertSig)
								uniqueVertUVIndexes[vertSig] = currentVertUVIndex
								uniqueVertUVs.append((faceId, vertexId, uvt[0], uvt[1], uv2t[0], uv2t[1], uv3t[0], uv3t[1]))
								triangleVertUVIndex = currentVertUVIndex
								currentVertUVIndex = currentVertUVIndex + 1

							triangleVertUVIndexes[currentTriangleId].append(triangleVertUVIndex)

						triangleMaterialIndexes.append(poly.material_index)
						currentTriangleId = currentTriangleId + 1

					# Write Vertices
					preservedTangsBinormsCount = 0
					calculatedTangsBinormsCount = 0

					# Write Vertices
					for uniqueVertUV in uniqueVertUVs:

						faceIndex = uniqueVertUV[0]
						vertexIndex = uniqueVertUV[1]
						vertex = mesh.vertices[vertexIndex]
						vertCoord = tuple(vertex.co)

						uv = (uniqueVertUV[2], uniqueVertUV[3])
						uv2 = (uniqueVertUV[4], uniqueVertUV[5])
						uv3 = (uniqueVertUV[6], uniqueVertUV[7])

						vertexFound = False

						vertNBT = FaceCornerBinormalsTangents[faceIndex][vertexIndex]
						vertNormal = (vertNBT[0], vertNBT[1], vertNBT[2])
						vertTangent = (vertNBT[3], vertNBT[4], vertNBT[5])
						vertBinormal = (vertNBT[6], vertNBT[7], vertNBT[8])

						filedata +='%.8f %.8f %.8f ' % (vertCoord[0] + position[0],  vertCoord[1] +  position[1], vertCoord[2] + position[2])
						filedata +='%.8f %.8f %.8f ' % (vertNormal[0], vertNormal[1], vertNormal[2])
						filedata +='%.8f %.8f %.8f ' % (vertTangent[0], vertTangent[1], vertTangent[2])
						filedata +='%.8f %.8f %.8f ' % (vertBinormal[0], vertBinormal[1], vertBinormal[2])

						filedata +='%.8f %.8f ' % (uv[0], 1 - uv[1])
						filedata +='%.8f %.8f ' % (uv2[0], 1 - uv2[1])
						filedata +='%.8f %.8f ' % (uv3[0], 1 - uv3[1])

						if vertexIndex in grannyVertexBoneWeights:
							vBoneWeightTuple = grannyVertexBoneWeights[vertexIndex]
						else:
							#raise "Error: Mesh has unweighted vertices!"
							vBoneWeightTuple = ([-1,-1,-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1,-1,-1,-1]) # Unweighted vertex - raise error

						filedata +='%d %d %d %d %d %d %d %d ' % (vBoneWeightTuple[0][0], vBoneWeightTuple[0][1],vBoneWeightTuple[0][2],vBoneWeightTuple[0][3], vBoneWeightTuple[0][4], vBoneWeightTuple[0][5],vBoneWeightTuple[0][6],vBoneWeightTuple[0][7]) # Bone Ids
						filedata +='%d %d %d %d %d %d %d %d\n' % (vBoneWeightTuple[1][0], vBoneWeightTuple[1][1],vBoneWeightTuple[1][2],vBoneWeightTuple[1][3], vBoneWeightTuple[1][4], vBoneWeightTuple[1][5],vBoneWeightTuple[1][6],vBoneWeightTuple[1][7]) # Bone Weights

					# Write Triangles
					filedata += "triangles\n"

					outputTriangles = []
					for triangle_id, triangle in enumerate(triangleVertUVIndexes): # mesh.polygons:
						materialIndex = triangleMaterialIndexes[triangle_id]
						outputTriangles.append((triangle[0],triangle[1],triangle[2], materialIndex))

					sortedOutputTriangles = sorted(outputTriangles, key=lambda triangle: triangle[3])

					for triangle in sortedOutputTriangles:
						filedata += '%i %i %i %i\n' % (triangle[0],triangle[1],triangle[2], triangle[3])

					print ("meshName: {}".format(meshName))
					print ("preservedTangsBinormsCount: {}".format(preservedTangsBinormsCount))
					print ("calculatedTangsBinormsCount: {}".format(calculatedTangsBinormsCount))

		filedata += "end"
		file.write(filedata)
		file.flush()
		file.close()
	except:
		filedata += "aborted!"
		file.write(filedata)
		file.flush()
		file.close()
		raise

	print ("End CN6 Export.")
	return ""

class export_cn6(bpy.types.Operator, ExportHelper):

	bl_idname = "export_shape.cn6"
	bl_label = "Export CN6 (.cn6)"
	bl_description= "Export a CivNexus6 .cn6 file"
	bl_options = {'PRESET'}

	filename_ext = ".cn6"
	filter_glob = StringProperty(default="*.cn6",options={'HIDDEN'})
	check_extension = True

	triangulate: BoolProperty(
			name="Triangulate",
			description="Triangulate meshes before exporting, this may result in changes to normals",
			default=True,
			)
	use_selection: BoolProperty(
			name="Selected Objects",
			description="Export only selected and visible objects",
			default=True,
			)

	def execute(self, context):
		print ("Export Filename: {}".format(self.filepath))
		do_export(context,
			self.filepath,
			self.triangulate,\
			self.use_selection,
			)
		return {'FINISHED'}

def menu_func(self, context):
	self.layout.operator(export_cn6.bl_idname, text="CivNexus6 (.cn6)")

def register():
	from bpy.utils import register_class
	register_class(export_cn6)

	bpy.types.TOPBAR_MT_file_export.append(menu_func)

def unregister():
	from bpy.utils import unregister_class
	unregister_class(export_cn6)

	bpy.types.TOPBAR_MT_file_export.remove(menu_func)

if __name__ == "__main__":
	register()