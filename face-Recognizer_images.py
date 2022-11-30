import face_recognition
import cv2
import os
import pickle
print(cv2.__version__)

j=0
Encodings=[]
Names=[]

scale=.5

with open('train.pkl','rb') as f:
	Names=pickle.load(f)
	Encodings=pickle.load(f)


font=cv2.FONT_HERSHEY_SIMPLEX


image_dir='/home/aaunano/PyPro/faceRecognizer/demoImages/unknown'
for root,dirs, files in os.walk(image_dir):
	for file in files:
		#print(root)
		print(file)
		testImagePath=os.path.join(root,file)
		testImage=face_recognition.load_image_file(testImagePath)
		facePositions=face_recognition.face_locations(testImage)
		allEncodings=face_recognition.face_encodings(testImage,facePositions)
		testImage=cv
image_dir='/home/aaunano/PyPro/faceRecognizer/demoImages/unknown'
for root,dirs, files in os.walk(image_dir):
	for file in fr (top,right,bottom,left),face_encoding in zip(facePositions, allEncodings):
			name='Unknown person'
			matches=face_recognition.compare_faces(Encodings,face_encoding)
			if True in matches:
				first_match_index=matches.index(True)
				name=Names[first_match_index]
			centery=(top+bottom)/2
			centerx=(right+left)/2
			cv2.rectangle(testImage,(left,top),(right,bottom),(0,0,255),2)


			print(name , " is at x:",centerx , ", y:" , centery)
		




