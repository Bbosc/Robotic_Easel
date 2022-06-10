import time, logging
from datetime import datetime
import threading, collections, queue, os, os.path
import deepspeech
import numpy as np
import pyaudio
import wave
import webrtcvad
from halo import Halo
from scipy import signal
import serial
import board
from adafruit_vl53l0x import VL53L0X


logging.basicConfig(level=20)

i2c = board.I2C()
time.sleep(1.0)
vl53 = VL53L0X(i2c)

RANGE = 35

class Audio(object):
    """Streams raw audio from microphone. Data is received in a separate thread, and stored in a buffer, to be read from."""

    FORMAT = pyaudio.paInt16
    # Network/VAD rate-space
    RATE_PROCESS = 16000
    CHANNELS = 1
    BLOCKS_PER_SECOND = 50


    def __init__(self, callback=None, device=None, input_rate=RATE_PROCESS, file=None):
        def proxy_callback(in_data, frame_count, time_info, status):
            #pylint: disable=unused-argument
            if self.chunk is not None:
                in_data = self.wf.readframes(self.chunk)
            callback(in_data)
            return (None, pyaudio.paContinue)
        if callback is None: callback = lambda in_data: self.buffer_queue.put(in_data)
        self.buffer_queue = queue.Queue()
        self.device = device
        self.input_rate = input_rate
        self.sample_rate = self.RATE_PROCESS
        self.block_size = int(self.RATE_PROCESS / float(self.BLOCKS_PER_SECOND))
        self.block_size_input = int(self.input_rate / float(self.BLOCKS_PER_SECOND))
        self.pa = pyaudio.PyAudio()

        kwargs = {
            'format': self.FORMAT,
            'channels': self.CHANNELS,
            'rate': self.input_rate,
            'input': True,
            'frames_per_buffer': self.block_size_input,
            'stream_callback': proxy_callback,
        }

        self.chunk = None
        # if not default device
        if self.device:
            kwargs['input_device_index'] = self.device
        elif file is not None:
            self.chunk = 320
            self.wf = wave.open(file, 'rb')

        self.stream = self.pa.open(**kwargs)
        self.stream.start_stream()

    def resample(self, data, input_rate):
        """
        Microphone may not support our native processing sampling rate, so
        resample from input_rate to RATE_PROCESS here for webrtcvad and
        deepspeech

        Args:
            data (binary): Input audio stream
            input_rate (int): Input audio rate to resample from
        """
        data16 = np.fromstring(string=data, dtype=np.int16)
        resample_size = int(len(data16) / self.input_rate * self.RATE_PROCESS)
        resample = signal.resample(data16, resample_size)
        resample16 = np.array(resample, dtype=np.int16)
        return resample16.tostring()

    def read_resampled(self):
        """Return a block of audio data resampled to 16000hz, blocking if necessary."""
        return self.resample(data=self.buffer_queue.get(),
                             input_rate=self.input_rate)

    def read(self):
        """Return a block of audio data, blocking if necessary."""
        return self.buffer_queue.get()

    def destroy(self):
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()

    frame_duration_ms = property(lambda self: 1000 * self.block_size // self.sample_rate)

    def write_wav(self, filename, data):
        logging.info("write wav %s", filename)
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.CHANNELS)
        # wf.setsampwidth(self.pa.get_sample_size(FORMAT))
        assert self.FORMAT == pyaudio.paInt16
        wf.setsampwidth(2)
        wf.setframerate(self.sample_rate)
        wf.writeframes(data)
        wf.close()


class VADAudio(Audio):
    """Filter & segment audio with voice activity detection."""

    def __init__(self, aggressiveness=3, device=None, input_rate=None, file=None):
        super().__init__(device=device, input_rate=input_rate, file=file)
        self.vad = webrtcvad.Vad(aggressiveness)

    def frame_generator(self):
        """Generator that yields all audio frames from microphone."""
        if self.input_rate == self.RATE_PROCESS:
            while True:
                yield self.read()
        else:
            while True:
                yield self.read_resampled()

    def vad_collector(self, padding_ms=300, ratio=0.75, frames=None):
        """Generator that yields series of consecutive audio frames comprising each utterence, separated by yielding a single None.
            Determines voice activity by ratio of frames in padding_ms. Uses a buffer to include padding_ms prior to being triggered.
            Example: (frame, ..., frame, None, frame, ..., frame, None, ...)
                      |---utterence---|        |---utterence---|
        """
        if frames is None: frames = self.frame_generator()
        num_padding_frames = padding_ms // self.frame_duration_ms
        ring_buffer = collections.deque(maxlen=num_padding_frames)
        triggered = False

        for frame in frames:
            if len(frame) < 640:
                return

            is_speech = self.vad.is_speech(frame, self.sample_rate)

            if not triggered:
                ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in ring_buffer if speech])
                if num_voiced > ratio * ring_buffer.maxlen:
                    triggered = True
                    for f, s in ring_buffer:
                        yield f
                    ring_buffer.clear()

            else:
                yield frame
                ring_buffer.append((frame, is_speech))
                num_unvoiced = len([f for f, speech in ring_buffer if not speech])
                if num_unvoiced > ratio * ring_buffer.maxlen:
                    triggered = False
                    yield None
                    ring_buffer.clear()


# list of possible moves : 
# BASICS : 
# Left (dir = 0), Right (dir =  1), High (dir = 2), Down (dir = 3)
# 
# Options :
# Extreme (option = 1) --> dir + option : go to maximal position in specified direction
# 
# Settings :
# Increase (settings = 1) --> increase default displacement size
# Decrease (settings = 2) --> decrease default displacement size
#
# Safety : 
# fin de course : returns a 4


def detect_keywords(txt):
  instruction_list = ['left','right',"up","down"]
  if ("left" in txt) or ("lift" in txt) : print("Will move to the left") ; direction = 0; return str(0)
  if ("right" in txt) or ("ride" in txt): print("Will move to the right"); direction = 1; return str(1)
  if ("hi " in txt)or ("high" in txt)  : print("Will move up"); direction = 2;           return str(2) 
  if ("down" in txt) or ("done" in txt)  : print("Will move down"); direction = 3;       return str(3)
  #return a 4 if out_of_limit
  if ("increase" in txt) : print("Increasing the default displacement size"); return str(5)
  if ("decrease" in txt) or ("degree" in txt) or ("decree" in txt) : print("Decreasing the default displacement size"); return str(6)
  
  if ("extreme" in txt) : print("Going to extreme " + instruction_list[direction]); return str(direction) + str(7)
  
  if ("and" in txt) or ("end" in txt) : print("Stop current movement"); return str(8)
  
  

def main(ARGS,text,arduino):

    # Load DeepSpeech model
    model = 'model.tflite'
    scorer = 'scorer.scorer'
    if os.path.isdir(model):
        #model_dir = ARGS.model
        model_dir = 'model.tflite'
        ARGS.model = os.path.join(model_dir, 'output_graph.pb')
        ARGS.scorer = os.path.join(model_dir, scorer)

    
    
    print('Initializing model...')
    logging.info("ARGS.model: %s", model)
    model = deepspeech.Model(model)
    if ARGS.scorer:
        logging.info("ARGS.scorer: %s", scorer)
        model.enableExternalScorer(scorer)

    # Start audio with VAD
    vad_audio = VADAudio(aggressiveness=ARGS.vad_aggressiveness,
                         device=ARGS.device,
                         input_rate=ARGS.rate,
                         file=ARGS.file)
    print("Listening (ctrl-C to exit)...")
    frames = vad_audio.vad_collector()

    # Stream from microphone to DeepSpeech using VAD
    spinner = None
    if not ARGS.nospinner:
        spinner = Halo(spinner='line')
    stream_context = model.createStream()
    wav_data = bytearray()
    
   
    for frame in frames:
      if vl53.range < RANGE : 
        fin_de_course = True
      #  print("there is a fin de course") 
      #  arduino.write((str(4).encode()))
      else :
        fin_de_course = False
        
      if frame is not None:
          if spinner: spinner.start()
          logging.debug("streaming frame")
          stream_context.feedAudioContent(np.frombuffer(frame, np.int16))
          if ARGS.savewav: wav_data.extend(frame)
      else:
          if spinner: spinner.stop()
          logging.debug("end utterence")
          if ARGS.savewav:
              vad_audio.write_wav(os.path.join(ARGS.savewav, datetime.now().strftime("savewav_%Y-%m-%d_%H-%M-%S_%f.wav")), wav_data)
              wav_data = bytearray()
          text = stream_context.finishStream()
          key = detect_keywords(text)
          if key is not None :
            i = 0
            #direction_banned[int(key)] = True if vl53.range < 30 else False
            while i < 2:  # necessary loop for the Arduino to get the message.   
              time.sleep(1)
              #if fin_de_course and direction_banned[int(key)] == True : key += "9" #change the message so that the arduino does not understand it --> it won't move
              arduino.write(key.encode())
              time.sleep(0.1)
              
              if  arduino.inWaiting()>0:
                  print("got the length")
                  answer=arduino.readline()
                  print(answer)
                  arduino.flushInput() #remove data after reading
                  break
              i+=1
          stream_context = model.createStream()
        
            

if __name__ == '__main__' : 
    DEFAULT_SAMPLE_RATE = 96000

    import argparse
    parser = argparse.ArgumentParser(description="Stream from microphone to DeepSpeech using VAD")

    parser.add_argument('-v', '--vad_aggressiveness', type=int, default=3,
                        help="Set aggressiveness of VAD: an integer between 0 and 3, 0 being the least aggressive about filtering out non-speech, 3 the most aggressive. Default: 3")
    parser.add_argument('--nospinner', action='store_true',
                        help="Disable spinner")
    parser.add_argument('-w', '--savewav',
                        help="Save .wav files of utterences to given directory")
    parser.add_argument('-f', '--file',
                        help="Read from .wav file instead of microphone")

    parser.add_argument('-m', '--model', 
                        help="Path to the model (protocol buffer binary file, or entire directory containing all standard-named files for model)")
    parser.add_argument('-s', '--scorer',
                        help="Path to the external scorer file.")
    parser.add_argument('-d', '--device', type=int, default=None,
                        help="Device input index (Int) as listed by pyaudio.PyAudio.get_device_info_by_index(). If not provided, falls back to PyAudio.get_default_device().")
    parser.add_argument('-r', '--rate', type=int, default=DEFAULT_SAMPLE_RATE,
                        help=f"Input device sample rate. Default: {DEFAULT_SAMPLE_RATE}. Your device may require 44100.")

    ARGS = parser.parse_args()
    if ARGS.savewav: os.makedirs(ARGS.savewav, exist_ok=True)
    txt = 0
    
    print('Running. Press CTRL-C to exit.')
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
        time.sleep(0.1) #wait for serial to open
        if arduino.isOpen():
            print("{} connected!".format(arduino.port))
            try:
             out_of_limit_signaled = False
             while True :
                if (vl53.range > RANGE) or out_of_limit_signaled:
                  main(ARGS,txt,arduino)
                  out_of_limit_signaled = False
                else :
                  arduino.write((str(4).encode()))
                  out_of_limit_signaled = True
              #main(ARGS,txt,arduino) 
              #return(main(ARGS,txt,arduino))
            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")

