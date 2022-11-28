import face_recognition
import cv2
import os
import pickle
print(cv2.__version__)

Encodings=[]
Names=[]

image_dir='/home/aaunano/PyPro/faceRecognizer/demoImages/known'
for root, dirs, files in os.walk(image_dir):
    print(files)
    for file in files:
        path=os.path.join(root,file)
        print(path)
        name=os.path.splitext(file)[0]
        print(name)
        person=face_recognition.load_image_file(path)
        encoding=face_recognition.face_encodings(person)[0]
        Encodings.append(encoding)
        Names.append(name)
print(Names)

with open('train.pkl','wb') as f:
    pickle.dump(Names,f)
    pickle.dump(Encodings,f)
