import face_recognition
import cv2
import os
import pickle
print (cv2.__version__)

Encodings=[]
Names=[]

scale=.5

with open('train.pkl','rb') as f:
	Names=pickle.load(f)
	Encodings=pickle.load(f)

font=cv2.FONT_HERSHEY_SIMPLEX
cam=cv2.VideoCapture(0)

while True:
	_,frame=cam.read()
	frameSmall=cv2.resize(frame,(0,0),fx=scale,fy=scale)
	frameRGB=cv2.cvtColor(frameSmall,cv2.COLOR_BGR2RGB)
	facePositions=face_recognition.face_locations(frameRGB,model='cnn')
	allEncodings=face_recognition.face_encodings(frameRGB,facePositions)
	for (top,right,bottom,left),face_encoding in zip(facePositions,allEncodings):
		name='Unknown person'
		matches=face_recognition.compare_faces(Encodings, face_encoding)
		if True in matches:
			first_match_index=matches.index(True)
			name=Names[first_match_index]
		top=int(top/scale)
		right=int(right/scale)
		bottom=int(bottom/scale)
		left=int(left/scale)
		centery=(top+bottom)/2
		centerx=(right+left)/2
		cv2.rectangle(frame,(left,top),(right,bottom),(0,0,255),2)
		cv2.putText(frame,name,(left,top-6),font,.75,(0,0,255),2)
		print(name , " is at x:",centerx , ", y:" , centery)
	cv2.imshow('Picture',frame)
	cv2.moveWindow('Picture',0,0)
	if cv2.waitKey(1)==ord('q'):
		break
cam.release()
cv2.destroyAllWindows()

