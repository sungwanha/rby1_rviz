from pynput import keyboard

def on_press(key):
    try:
        # 일반 문자 키(a, b, c 등)인 경우
        print(f"눌린 키: {key.char}")
    except AttributeError:
        # 특수 키(F12, Space, Ctrl 등)인 경우
        print(f"눌린 특수 키: {key}")

with keyboard.Listener(on_press=on_press) as listener:
    print("발판을 밟아서 어떤 키가 뜨는지 확인하세요... (종료: Ctrl+C)")
    listener.join()