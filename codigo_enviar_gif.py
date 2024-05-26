import subprocess
import os

def play_gif_with_mpv(gif_path):
    # Certifique-se de que a variável DISPLAY está configurada para :0
    os.environ['DISPLAY'] = ':0'

    # Comando para executar o mpv com um método de saída de vídeo específico
    command = ['mpv', '--fullscreen', '--loop=inf', '--vo=x11', gif_path]

    # Executa o comando
    subprocess.run(command)

if __name__ == '__main__':
    gif_path = '/home/lisa/Lisa/lisa_desktop/InLove.gif'
    play_gif_with_mpv(gif_path)
