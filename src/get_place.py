# 指定した行のbool配列のうちTrueが最も連続する区間の中央が配列の中央(w/2)からどれ程ずれているかを返します．
def get_place(lis, height):
    h = len(lis)
    w = len(lis[0])
    assert(0 <= height and height < h)

    segment = list()
    left = 0
    right = 0

    while right < w:
        if lis[height][left] == False:
            left += 1
        elif lis[height][right] == False:
            segment.append((left, right))
            left = right
        right += 1
    if lis[height][left] == True:
        segment.append((left, right))

    assert(len(segment) >= 1)

    max_len = -1
    max_segment = (-1, -1)

    for cur_segment in segment:
        if cur_segment[1] - cur_segment[0] > max_len:
            max_len = cur_segment[1] - cur_segment[0]
            max_segment = cur_segment

    return (max_segment[0] + max_segment[1] - w) / 2

if __name__ == "__main__":
    # test
    lis = [[False, False, True, True, False, True, True, True, False]]
    print(get_place(lis, 0))
        