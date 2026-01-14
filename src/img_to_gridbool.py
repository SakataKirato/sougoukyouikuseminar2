import cv2
# import numpy

# 動かす環境に合わせて調節して下さい．
c_value = 50

#入力で受け取った画像を指定したピクセルで平均を取りT/Fの二次元配列にして返す.
def img_to_grid(image) -> list[list[bool]]:

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray_image.shape

    prosessed_image = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 101, c_value)

    result = [[0] * w for _ in range(h)]

    for i in range(h):
        for j in range(w):
            if prosessed_image[i][j] == 0 :
                result[i][j] = True
            else:
                result[i][j] = False

    return result

#テスト用
if __name__ == "__main__":
    print(cv2.__version__)
    image = cv2.imread('data_for_test/image.jpg')
    lis = img_to_grid(image)

    for i in range(len(lis)):
        for j in range(len(lis[i])):
            if lis[i][j]==True:
                print('#', end='')
            else:
                print('.', end='')
        print()