import cv2


def funcCan(thresh1=0):
    thresh1 = cv2.getTrackbarPos('thresh1', 'canny')
    thresh2 = cv2.getTrackbarPos('thresh2', 'canny')
    edge = cv2.Canny(img, thresh1, thresh2)
    cv2.imshow('canny', edge)


if __name__== '__main__':
    original = cv2.imread("./image_1_colour.png", 0)
    img = original.copy()
    img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
    img = cv2.GaussianBlur(img, (5, 5), 0)

    cv2.namedWindow('canny')

    thresh1 = 100
    thresh2 = 1
    cv2.createTrackbar('thresh1', 'canny', thresh1, 255, funcCan)
    cv2.createTrackbar('thresh2', 'canny', thresh2, 255, funcCan)
    funcCan()
    # cv2.imshow('Frame', original)

    cv2.waitKey(0)

cv2.destroyAllWindows()


