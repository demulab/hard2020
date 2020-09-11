#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, copy, rospy, time
import subprocess, ngram
import numpy as np
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechRecog:
    # コンストラクタ
    def __init__(self):
        rospy.init_node('speech_recog',anonymous=True)  # ノードの初期化                                                 

        self.recog_transcript = []  #　認識結果
        self.question_all_no  = 10  #　全問題数
        self.question_no      = 3   #  質問の回数
        self.answer_count     = 0   #  回答数

        # 質問文
        self.sentence_all = [
            "Who are the inventors of the C programming language?", #1
            "Who is the inventor of the Python programming language?", #2
            "Where does the term computer bug come from?", #3
            "What is the name of the round robot in the new Star Wars movie?", #4
            "What did Alan Turing create?", #5
            "What Apollo was the last to land on the moon?", #6
            "Who developed the first industrial robot?", #7
            "Which company makes ASIMO?", #8
            "How many people live in the Germany?", #9
            "What city is the capital of the Germany?" #10
        ]

        # 解答文
        self.sentence_answer = [
            "Ken Thompson and Dennis Ritchie", #1
            "Guido van Rossum", #2
            "From a moth trapped in a relay", #3
            "BB-8", #4
            "Many things like Turing machines and the Turing test", #5
            "Apollo 17", #6
            "The American physicist Joseph Engelberg. He is also considered the father of robotics.", #7
            "Honda", #8
            "A little over 80 million", #9
            "Berlin" #10
        ] 
  
        # subscriber
        rospy.Subscriber('/create1/speech_result',SpeechRecognitionCandidates, self.speech_recog_cb)
    
        # publisher
        self.speak_pub = rospy.Publisher('/tts', String, queue_size=10)

        # ウエルカムメッセージ
        time.sleep(1) # この間を入れないと次の文を発話しない
        self.speak("Welcome to the Hard Question answering system. Please ask me a question after a beep sound", 8)


    # 音声認識コールバック関数。認識した結果が引数recogに入る。
    def speech_recog_cb(self,recog):
        self.recog_transcript = recog.transcript # 認識結果の文
        self.confidence       = recog.confidence # 認識結果の信頼度


    # 発話
    def speak(self, sentence, sleep_time=3.5):
        self.speak_pub.publish(sentence)
        time.sleep(sleep_time) # pubulishした後に少しスリープが必要。秒数は調整すること
        

    # 質問のタイミングを示すビープ音
    def beep(self):
        time.sleep(0.3)
        args = ['/usr/bin/paplay', '/usr/share/sounds/freedesktop/stereo/complete.oga']
        subprocess.call(args)


    #　メイン関数
    def main(self):
        rate = rospy.Rate(1)

        time.sleep(1)       # 少し間を置く [s]
        speech_recog.beep() # 回答を促すビープ音

        while not rospy.is_shutdown():
            max_k   = 0
            max_no  = 0
            max_val = 0

            time.sleep(1)
            rospy.loginfo(self.recog_transcript)

            # 文章間の類似度を計算する。音声認識結果は完全ではないのでに近い質問があるか調べる。
            # 文章間の類似度を計算する方法はいろいろあるが、ここではN-gramを使う手法を用いている。
            # recog_transcriptには複数の音声認識結果の文章が格納されているので、それら全てと
            # 全質問文の類似度を計算して、一番類似度の高い文章を質問文としている。
            if len(self.recog_transcript) > 0: # 音声認識結果がある場合だけ処理。
                for i in range(0, self.question_all_no):
                    for k in range(0, len(self.recog_transcript)):
                        rospy.loginfo(self.recog_transcript[k])
                        val = ngram.NGram.compare(self.recog_transcript[k], self.sentence_all[i])
                        if val >=  max_val:
                            max_k   = k
                            max_no  = i
                            max_val = val
                            rospy.loginfo('max_val=%s', str(max_val))
                            
            if max_val >= 0.3: # 信頼度の最大値が0.3以上なら。この値は調整必要。
                self.speak('Answer', 1)
                self.speak(self.sentence_answer[max_no], 5)
                
                rospy.loginfo("answer[%d]:%s",max_no, self.sentence_answer[max_no])
                print(self.confidence)
                del self.recog_transcript[:]
                time.sleep(3)
                self.answer_count = self.answer_count + 1
                if self.answer_count < self.question_no:
                    speech_recog.beep() # 回答を促すビープ音
                    time.sleep(1)       # 少し間を置く [s]
                else:
                    self.speak('Congratulations! You are a quiz king!', 3)
                    sys.exit() # プログラム終了

            rate.sleep()
            

if __name__ == '__main__':
    try:
        speech_recog = SpeechRecog()
        speech_recog.main()
    except rospy.ROSInterruptException: pass
    
    

