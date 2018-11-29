import numpy as np
import cv2
import IPython
import time


def divide_image(img, nrow, ncol, channel):
    row_pix = img.shape[0]
    col_pix = img.shape[1]
    row_size = int(row_pix/nrow)
    col_size = int(col_pix/ncol)

    data = []
    new_img = img.copy()
    for row in range(nrow):
        for col in range(ncol):
             patch = img[row*row_size:(row+1)*row_size,col*col_size:(col+1)*col_size,channel]
             mean_patch = np.mean(patch)
             center_of_patch = [int((row*row_size+(row+1)*row_size)/2), int((col*col_size+(col+1)*col_size)/2)]
             cv2.putText(new_img,str(int(mean_patch)),(center_of_patch[1],center_of_patch[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(255,255,255),1,cv2.LINE_AA)
             data.append([center_of_patch[0], center_of_patch[1],mean_patch])

    #nupied = np.asarray(data)
    #np.savetxt('for_matlab2.csv',nupied, delimiter =',')
    #time.sleep(100)


    return new_img, data




cap = cv2.VideoCapture('/dev/video0')
cap.set(3,1280)
cap.set(4,720)

full_data = []
counter = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # operations
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h = hsv.copy()
    h[:,:,1] = 0
    h[:,:,2] = 0

    s = hsv.copy()
    s[:,:,0] = 0
    s[:,:,2] = 0

    v = hsv.copy()
    v[:,:,0] = 0
    v[:,:,1] = 0


    # Display the shits
    cv2.imshow('image',frame)
    cv2.imshow('hsv', hsv)
    cv2.imshow('h', h)
    cv2.imshow('s', s)
    cv2.imshow('voltage', v)
    result,dk = divide_image(v, 50,50,2)
    #if counter%100 ==0:
    #    print 'capturing'
    #    full_data.append(data)
    cv2.imshow('result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #if len(full_data) ==4:
    #    break
    #counter = counter +1

#stacked = np.concatenate(full_data)
#nupied = np.asarray(stacked)
#np.savetxt('for_matlab2.csv',nupied, delimiter =',')
#print 'finished'


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
