import cv2

# 加载一张图片（替换'image_path.jpg'为你的图片文件路径）
image_path = 'image_path.jpg'
img = cv2.imread(image_path)

if img is not None:
    cv2.imshow('Test Image', img)
    cv2.waitKey(0)  # 等待按键后关闭窗口
    cv2.destroyAllWindows()
else:
    print("Error: 图片未能加载，请检查路径是否正确")
