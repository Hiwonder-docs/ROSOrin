# 19. Large AI Model Courses

## 19.1 Large Models Basic Courses

### 19.1.1 Large Language Model Courses

* **Overview of Large Language Model**

A Large Language Model (LLM) is an advanced artificial intelligence model developed to comprehend and generate human language.

<img  src="../_static/media/chapter_19/section_1.1/01/image2.png" style="width:500px" class="common_img"/>

(1) Basic Concept

A Large Language Model (LLM) is a deep learning model trained on extensive text data, designed to either generate natural language text or comprehend the meaning of language. LLM is capable of performing various natural language tasks, such as text classification, question answering, and dialogue, making them a crucial step toward achieving advanced artificial intelligence. Unlike smaller models, LLM leverages a similar Transformer architecture and pretraining objectives (like Language Modeling), but are distinguished by their larger model size, greater training data, and enhanced computational resources.

(2) Features

**Massive Scale:** LLM are characterized by their vast parameter sizes, often reaching billions or even trillions of parameters. This immense scale allows them to capture a wealth of linguistic knowledge and complex syntactic structures.

**Pretraining and Fine-tuning:** LLM utilize a two-stage learning process: pretraining and fine-tuning. Initially, they are pretrained on large-scale, unlabeled text data, learning general language representations and knowledge. Subsequently, they are fine-tuned using labeled data to specialize in specific tasks, allowing them to excel across a wide range of NLP applications.

**Contextual Awareness:** LLM demonstrate exceptional contextual awareness, with their ability to understand and generate language deeply dependent on preceding text. This enables them to perform exceptionally well in tasks like dialogue, article generation, and contextual comprehension.

**Multilingual Capabilities:** LLM support multiple languages, extending beyond just English. This multilingual proficiency enables them to power cross-lingual and cross-cultural applications, enhancing their versatility and global reach.

**Multimodal Support:** Some LLMs have expanded to handle multimodal data, including text, images, and speech. This capability allows them to understand and generate content across various media types, opening up more diverse application possibilities.

**Emergent Properties:** LLM exhibit remarkable emergent properties, where performance improvements become apparent in large models but are absent in smaller ones. This makes them adept at handling more complex tasks and challenges.

**Cross-domain Applications:** LLM have been widely adopted across numerous fields, including text generation, machine translation, information retrieval, summarization, chatbots, and virtual assistants. Their influence is profound, impacting both daily life and work in significant ways.

**Ethical and Risk Considerations:** While LLM showcase impressive capabilities, they also raise important ethical and risk-related concerns, such as the potential generation of harmful content, privacy violations, and cognitive biases. As such, the development and deployment of LLM must be approached with careful consideration and caution.

(3) Working Principle

Large Language Model (LLM) are built on deep learning principles and are trained using massive datasets and substantial computational resources to develop neural networks with billions of parameters. Through iterative training and parameter optimization, these models learn to perform a wide range of tasks with high accuracy. The "**large**" in LLM reflects their immense scale—encompassing a vast number of parameters, extensive training data, and significant computational demands. This scale enables advanced models to achieve superior generalization capabilities and deliver increasingly accurate results, even in highly specialized domains.

Today, some of the most popular applications revolve around generative AI, such as language generation tools (e.g., ChatGPT) and image generation platforms (e.g., Midjourney). At the core of these applications is the concept of generation—the model's ability to predict and produce coherent content based on a given input.

(4) Application Scenarios

① Text Generation

Large Language Models are capable of generating diverse types of text, including news articles, stories, poems, and more. These capabilities make them well-suited for applications in content creation, creative writing, and automated storytelling.

② Text Classification

Large Language Models can classify text into various categories, such as sentiment analysis and topic identification. These capabilities are especially valuable in scenarios like public opinion analysis, information retrieval, and content moderation.

③  Machine Translation

Large Language Models excel at machine translation, enabling the conversion of text from one language to another. These capabilities are essential for cross-language communication, localization, and global collaboration.

④ Question-Answering Systems

Large Language Models can be used to build question-answering systems that respond to user queries. These applications are particularly valuable in areas such as intelligent customer support, knowledge retrieval, and information lookup.

<p id="anchor19.1.1.2"></p>

* **Large Language Model Deployment**

> [!NOTE]
>
> This section outlines the steps to register on the official OpenAI website and obtain the API key for the Large Language Model.

(1) OpenAI Account Registration and Setup

① Copy the following URL: <https://platform.openai.com/docs/overvie>

Open the OpenAI website and click on the "**Sign Up**" button in the top right corner.

<img  src="../_static/media/chapter_19/section_1.1/02/image2.png" style="width:500px" class="common_img"/>

② Follow the prompts to register and log in using your Google, Microsoft, or Apple account.

<img  src="../_static/media/chapter_19/section_1.1/02/image3.png" style="width:500px" class="common_img"/>

③ Click on the settings icon, then select Billing, followed by Payment Methods, to link your payment method. Recharge your account to purchase tokens.

<img  src="../_static/media/chapter_19/section_1.1/02/image4.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.1/02/image5.png" style="width:500px" class="common_img"/>

④ After completing the setup, click on API Keys, then select Create New Secret Key. Follow the prompts to fill in the required information. Once the key is created, make sure to save it for future use.

<img  src="../_static/media/chapter_19/section_1.1/02/image6.png" style="width:200px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.1/02/image7.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.1/02/image8.png" style="width:500px" class="common_img"/>

⑤ With these steps, the large model has been successfully created and deployed. You can now use the API in the upcoming lessons.

(2) OpenRouter Account Registration and Setup

① Copy the following URL: <https://openrouter.ai/>

Open the webpage in your browser and click "**Sign In**". Register using your Google account or another available login option.

<img  src="../_static/media/chapter_19/section_1.1/02/image9.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.1/02/image10.png" style="width:400px" class="common_img"/>

② After logging in, click the icon in the top-right corner and select **"Credits"** to link your payment method.

<img  src="../_static/media/chapter_19/section_1.1/02/image11.png" style="width:400px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.1/02/image12.png" style="width:500px" class="common_img"/>

③ To create an API key, go to **"API Keys"**, then click **"Create Key"**. Follow the prompts to complete the process. Once the key is generated, make sure to save it for future use.

<img  src="../_static/media/chapter_19/section_1.1/02/image13.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.1/02/image14.png" style="width:500px" class="common_img"/>

④ At this point, the large model is successfully created and deployed. You can now use the API in the upcoming lessons.

<p id="anchor19.1.1.3"></p>

* **Large Language Model Accessing**

> [!NOTE]
>
> To proceed with this section, you will need to register on the appropriate website and obtain the API key for the large model (please refer to the file ["**19.1.1 Large Language Model Courses -> Large Language Model Deployment**"](#anchor19.1.1.2)).

It is important to ensure a stable network connection for the development board. For optimal performance, we also recommend connecting the main controller to a wired network for enhanced stability.

<img  src="../_static/media/chapter_19/section_1.1/03/image2.png" style="width:300px" class="common_img"/>

(1) Environment Configuration

> [!NOTE]
>
> If you have purchased a robot from our company with built-in large model functionality, the environment is already pre-configured in the robot's image. You can directly proceed to Section 3 of this document to configure the API key.

Install Vim and Gedit by running the corresponding commands. Install the necessary software packages and audio libraries required for PyAudio.

```
sudo apt update
```

```
sudo apt install vim
```

```
sudo apt install gedit
```

```
sudo apt install python3 python3-pip python3-all-dev python3-pyaudio portaudio11-dev libsndfile1
```

<img  src="../_static/media/chapter_19/section_1.1/03/image3.png" style="width:500px" class="common_img"/>

(2) Importing the Large Model Program Directory

① In this section, locate the '[Appendix -> Source Code](https://drive.google.com/drive/folders/1Na86By9er9Jj1_1YXz3sxAwePrIgSUcN?usp=sharing)' folder within the same directory as this tutorial document.

<img  src="../_static/media/chapter_19/section_1.1/03/image4.png" style="width:500px" class="common_img"/>

② Using the MobaXterm remote connection tool (as outlined in the '5.5 Remote Access and File Transfer' tutorial), drag the folder into the root directory of the main controller. The software installation package can be found in the '[Appendix -\> Remote Access and File Transfer](https://drive.google.com/drive/folders/17mfRH9lmP9OYO4_LAyzkRnHfytqRYldJ?usp=sharing)' directory.

<img  src="../_static/media/chapter_19/section_1.1/03/image5.png" style="width:500px" class="common_img"/>

③ Next, execute the command to navigate to the **'speech_pkg' directory**.

```
cd ~/large_models/speech_pkg/
```

④ Execute the following commands to install the necessary third-party libraries.

```
pip3 install -r requirements.txt --break-system-packages
```

```
pip3 install dashscope --break-system-packages
```

```
pip3 install opencv-python --break-system-packages
```

```
pip3 install sympy==1.13.1 --break-system-packages
```

```
pip3 install torch --break-system-packages
```

⑤ Then, use the command in the terminal to navigate to the **'speech'** directory.

```
cd ~/large_models/speech_pkg/speech
```

⑥ Run the command to list the files in the **'speech'** directory.

```
ls
```

<img  src="../_static/media/chapter_19/section_1.1/03/image13.png" style="width:500px" class="common_img"/>

⑦ Depending on the type of main controller and Python version you're using, switch to the appropriate folder for packaging and distribution. This tutorial uses the Jetson Orin controller as an example.

| **Type of main controller** | **Python version** |
| --------------------------- | ------------------ |
| jetson_nano                 | 3.6                |
| jetson_orin                 | 3.10               |
| rpi5                        | 3.11               |
| rpi5_docker                 | 3.8                |

⑧ Execute the following command to navigate to the Jetson Orin folder.

```
cd jetson_orin/
```

⑨ Enter the command to copy the 'speech.so' file to the parent directory.

```
cp -r speech.so ..
```

⑩ Enter the command to navigate to the parent directory.

```
cd ../..
```

⑪ Execute the command to package the speech file for the Python environment.

```
pip3 install .
```

⑫ Enter the command to install and update the OpenAI Python library.

```
pip3 install openai -U
```

(3) Key Configuration

① Open the terminal and enter the following command to navigate to the directory for configuring the large model keys:

```
cd ~/large_models
```

② Then, open the configuration file by running:

```
vim config.py
```

③ Once the file is open, configure the OpenAI and OpenRouter keys by filling in the llm_api_key and vllm_api_key parameters, respectively (you can obtain these keys from the '[19.1.1 Large Language Model Courses -> Large Language Model Deployment](#anchor19.1.1.2)' course).

<img  src="../_static/media/chapter_19/section_1.1/03/image21.png" style="width:500px" class="common_img"/>

For instance, copy the key created in Section 1.2 of this chapter and paste it into the appropriate field. To paste the key, place the cursor between the quotation marks, hold the **"Shift"** key, right-click, and select **"Paste"** .

> [!NOTE]
>
> Do not mix keys from different models, as this may cause the functionality to malfunction

<img  src="../_static/media/chapter_19/section_1.1/03/image22.png" style="width:500px" class="common_img"/>

④  After pasting, press the **'Esc'** key, then type the following command and press Enter to save the file:

```
:wq
```

(4) Running the Demo Program

Once the keys are configured, you can run the demo program (openai_llm_demo.py) to experience the text generation capabilities of the large model. For example, the program's prompt might be: 'Write a 50-word article about how technology is changing life.'

<img  src="../_static/media/chapter_19/section_1.1/03/image24.png" style="width:500px" class="common_img"/>

① To run the demo, enter the following command in the terminal:

```
python3 openai_llm_demo.py
```

② After running the program, the output will appear as shown in the image below.

<img  src="../_static/media/chapter_19/section_1.1/03/image26.png" style="width:500px" class="common_img"/>

* **Semantic Understanding with Large Language Model**

Before starting this section, make sure you have completed the API key configuration outlined in the file [19.1.1 Large Language Model Courses -\> Large Language Model Accessing](#anchor19.1.1.3).

In this lesson, we'll use a large language model to analyze and summarize short passages of text.

(1) Start by opening a new terminal window, then navigate to the large model project directory:

```
cd large_models/
```

(2) Next, run the demo program with the following command:

```
python3 openai_llm_nlu_demo.py
```

(3) As shown in the output, the model demonstrates strong summarization abilities.

<img  src="../_static/media/chapter_19/section_1.1/01/image4.png" style="width:500px" class="common_img"/>

(4) The result matches the prompt defined in the program — where a passage of text is provided to the model, and it generates a concise summary.

<img  src="../_static/media/chapter_19/section_1.1/01/image5.png" style="width:500px" class="common_img"/>

* **Emotional Perception with Large Language Model**

To proceed with this section, ensure that you have completed the API key configuration as described in the file [19.1.1 Language Model Courses -> Large Language Model Accessing](#anchor19.1.1.3).

In this lesson, we will use a large language model to assess its ability to perceive emotions based on descriptive words. We'll provide the model with emotional expressions and evaluate its response.

(1) Start by opening a new terminal window, then navigate to the large model project directory:

```
cd large_models/
```

(2) Next, run the demo program with the following command:

```
python3 openai_llm_er_demo.py
```

(3) From the output, you will see that the model successfully identifies and understands the emotions conveyed, providing a text-based response accordingly.

<img  src="../_static/media/chapter_19/section_1.1/01/image6.png" style="width:500px" class="common_img"/>

(4) In this program, we send two emotional expressions to the model: the first is an expression of sadness, **"So Sad"**. After the model responds, we then send an expression of happiness, "**Ha Ha**", and observe how the model reacts.

<img  src="../_static/media/chapter_19/section_1.1/01/image7.png" style="width:500px" class="common_img"/>

### 19.1.2 Large Speech Model Courses

* **Overview of Large Speech Model**

(1) What is a Large Speech Model?

A Speech Large Model (LSM) refers to a machine learning model that uses deep learning techniques to process and understand speech data. These models can be applied in a variety of tasks, such as speech recognition, speech synthesis, speech translation, and emotional analysis of speech. The design and training of these models typically require large amounts of speech data and substantial computational resources, which is why they are referred to as "**large models**".

(2) Why Do We Need Large Speech Model?

With the advancement of artificial intelligence and deep learning, traditional speech processing methods face many limitations. Large models leverage vast amounts of data and deep neural networks to learn and understand the complex features within speech, thereby improving the accuracy and naturalness of speech recognition and generation.

Their advantages include:

① High Accuracy: They maintain a high recognition rate even in noisy environments and with various accents.

② Naturalness: Speech generated by synthesis models is more natural, closely resembling human speech.

③ Versatility: These models support a wide range of languages and tasks, such as multilingual speech recognition, speech-to-text (STT), text-to-speech (TTS), and emotion recognition.

(3) Development of Speech Recognition Technology

Word-Level Speech Recognition: At this stage, speech recognition systems could only recognize individual words

Phrase-Level Speech Recognition: With the expansion of data and advancements in algorithms, speech recognition systems gradually gained the ability to recognize longer phrases, such as "**Please turn on my computer**".

Sentence-Level Speech Recognition: In recent years, with the emergence of AI large models, speech recognition systems have become capable of recognizing entire sentences and understanding their underlying meaning.

(4) Differences Between Large Speech Model and Traditional Speech Processing Technologies

① Processing Methods

Traditional Speech Processing Technologies: These typically rely on manual feature extraction and shallow models, such as Gaussian Mixture Models (GMM) and Hidden Markov Models (HMM), to process speech signals.

Large Speech Model: These use end-to-end learning, directly mapping raw speech waveforms to target outputs (such as text or another speech signal), reducing the reliance on manual feature extraction. They are typically based on deep learning architectures, such as Convolutional Neural Networks (CNN), Recurrent Neural Networks (RNN), and Transformers.

② Model Complexity

Traditional Speech Processing Technologies: These models are relatively simple, with fewer parameters.

Large Speech Model: These models have complex structures and a large number of parameters, enabling them to capture more subtle speech features and contextual information.

③ Recognition Capability

Traditional Speech Processing Technologies: These are highly adaptable to specific scenarios and conditions, but their recognition capability is limited when encountering new, unseen data.

Large Speech Model: Due to their large number of parameters and powerful learning ability, they offer superior recognition capabilities and can adapt to a wider variety of speech data and environments.

④Training Data Requirements

Traditional Speech Processing Technologies: These typically require less data for training, but the data must be highly annotated and of high quality.

Large Speech Model: These require vast amounts of training data to fully learn the complexities of speech, often necessitating large quantities of annotated data or the use of unsupervised/self-supervised learning methods.

(5) Core Technologies of Speech Large Model

① Automatic Speech Recognition (ASR)

ASR is the technology that converts human speech into text. The core steps of a speech recognition system include feature extraction, acoustic modeling, and language modeling.

② Text-to-Speech (TTS)

TTS is the technology that converts text into speech. Common speech synthesis models include the Tacotron series, FastSpeech, and VITS.

③ Speech Enhancement and Noise Reduction

Speech enhancement techniques are used to improve the quality of speech signals, typically for eliminating background noise and echoes. This is crucial for speech recognition applications in noisy environments.

(6) Applications of Speech Large Model

Intelligent Voice Assistants: For instance, Amazon Alexa and Google Home, which engage with users through voice interactions.

Customer Service Chatbots: In the customer service sector, speech large models assist businesses in enhancing service efficiency by swiftly processing customer inquiries through speech recognition technology, enabling 24/7 support.

Healthcare: Helping doctors with medical record-keeping, thus improving work efficiency.

Speech-to-Text: Speech large models excel in converting speech to text, offering accurate recognition and transcription in a variety of contexts. They are widely used in applications such as meeting transcription and subtitle generation.

* **Voice Device Introduction and Testing**

(1) Device Overview

① 6-Microphone Circular Array

Introduction：

The 6-Microphone Circular Array is a high-sensitivity, high signal-to-noise ratio microphone board. It features six analog silicon microphones arranged in a circular pattern. When paired with a main control board, it supports high-performance Acoustic Echo Cancellation (AEC), environmental noise reduction, and factory-level voice pickup from up to 10 meters.

<img  src="../_static/media/chapter_19/section_1.2/02/image2.png" style="width:300px" class="common_img"/>

Features and Specifications：

**Operating Voltage:** 3.3V (typical)

**Operating Current:** 0.8mA (typical)

**Operating Temperature:** -20°C (min), 25°C (typical), 70°C (max)

**Operating Humidity:** Up to 95% relative humidity (max)

(1) Recording and Playback Test

The following demonstration uses the Raspberry Pi 5 as an example. The connection and testing steps are also applicable to other compatible devices such as the Jetson series:

① Connection Illustration and Detection

<img  src="../_static/media/chapter_19/section_1.2/02/image3.png" style="width:500px" class="common_img"/>

If the main controller is a Raspberry Pi, you can use VNC remote desktop access (refer to the appendix: Remote Access and File Transfer) to log into the Raspberry Pi system. Once connected, check the upper right corner of the desktop for microphone and speaker icons. As shown in the image below, the presence of these icons indicates a successful connection.

<img  src="../_static/media/chapter_19/section_1.2/02/image4.png" style="width:500px" class="common_img"/>

If you're using a NVIDIA Jetson device, you can connect via the NoMachine remote access tool. After logging in, check the upper right corner of the system interface for the speaker icon to confirm successful detection.

<img  src="../_static/media/chapter_19/section_1.2/02/image5.png" style="width:500px" class="common_img"/>

② Recording Test

Next, open a new terminal window and enter the following command to check the available recording devices. Note that the -l option is a lowercase "**L**". You should see the card number (card) listed—for example, card 0. This is just an example; please refer to your actual query result.

```
arecord -l
```

<img  src="../_static/media/chapter_19/section_1.2/02/image6.png" style="width:500px" class="common_img"/>

Then, use the following command to start recording. Replace the red-marked card number (hw:0,0) with the actual number you found in the previous step:

```
arecord -D hw:0,0 -f S16_LE -r 16000 -c 2 test.wav
```

This will create a test.wav audio file in the current directory.

You can record a short 5-second sample, then press Ctrl + C to stop the recording.

③ Playback Test

After the recording is complete, you can check whether the audio file was successfully created by listing the contents of the current directory:

```
ls
```

<img  src="../_static/media/chapter_19/section_1.2/02/image8.png" style="width:500px" class="common_img"/>

If test.wav appears in the list, the recording was successful. To play back the recording, use the following command:

```
aplay test.wav
```

* **Voice Wake-Up**

In this lesson, we'll learn how to use a large speech model to activate the voice device by speaking a predefined wake word through a program.

(1) WonderEcho Pro Wake-Up

Device Check：

To proceed, we need to identify the USB device name assigned to the connected WonderEcho Pro or voice device (hereafter referred to as the voice device). Please follow the steps below carefully. 

> [!NOTE]
>
> Do not connect any other USB devices during this process to avoid confusion.

① First, disconnect the voice device, then open a terminal and run the following command:

```
ll /dev | grep USB
```

② Next, reconnect the voice device to the USB port on your main board and run the same command again:

```
ll /dev | grep USB
```

③ You should now see a newly listed USB port, such as ttyCH341USB1.  

Please take note of this device name—it may vary depending on the main controller being used.

<img  src="../_static/media/chapter_19/section_1.2/03/image3.png" style="width:500px" class="common_img"/>

Wake-Up Test：

① To begin, update the port number used in the program by editing the script. You'll also need to uncomment the line for the port you're using and comment out any unused ports.

```
vim wakeup_demo.py
```

Press i to enter edit mode and make the necessary changes as shown below (update the port number accordingly and adjust comments as needed).

<img  src="../_static/media/chapter_19/section_1.2/03/image4.png" style="width:500px" class="common_img"/>

Once the changes are complete, press ESC, then type :wq and press Enter to save and exit the editor.

② Next, return to the system interface and run the wake-up demo using the command below. Speak the wake word **"HELLO HIWONDER"** clearly toward the WonderEcho Pro voice device.  

If the output includes **"keyword detect"**, it indicates that the firmware has been successfully flashed and the wake word is functioning correctly.

```
python3 ~/large_models/wakeup_demo.py
```

<img  src="../_static/media/chapter_19/section_1.2/03/image5.png" style="width:500px" class="common_img"/>

(2) 6-Microphone Circular Array

As with the WonderEcho Pro, you can connect the 6-Microphone Circular Array to your main board (Raspberry Pi or NVIDIA Jetson) using a Type-C to USB cable.

Device Check:

For Jetson users, connect to the Jetson system using the NoMachine remote access tool. Once connected, check the desktop interface.  

If the 6-Microphone Circular Array icon appears on the left side of the desktop, it indicates the device has been successfully recognized.

Wake-Up Test:

① Open a new terminal window and run the following command to edit the wakeup_demo.py script:

```
vim ~/large_models/wakeup_demo.py
```

② Press i to enter edit mode.

③ Update the port to match the device port number you previously identified. Comment out the WonderEcho Pro configuration (add \# at the beginning of the corresponding line), and uncomment the line using the voice device on line 11 as the input device (see red box in the referenced image).

<img  src="../_static/media/chapter_19/section_1.2/03/image7.png" style="width:500px" class="common_img"/>

④ Press ESC to return to command mode, then type :wq and press Enter to save and exit.

<img  src="../_static/media/chapter_19/section_1.2/03/image8.png" style="width:500px" class="common_img"/>

⑤ In the terminal, run the wake-up program with the following command:

```
python3 ~/large_models/wakeup_demo.py
```

⑥ After about 30 seconds of initialization, speak the wake word **"hello hiwonder"** to test the device.

<img  src="../_static/media/chapter_19/section_1.2/03/image10.png" style="width:500px" class="common_img"/>

(3) Brief Program Overview

This is a Python-based wake word detection script that utilizes the speech module to process audio input and detect a specific wake word (e.g., "**HELLO_HIWONDER**").

Source Code Path: `/home/ubuntu/large_models/wakeup_demo.py`

Importing Required Modules

{lineno-start=5}

```
import os
import time
from speech import awake
```

`os`: Used for handling file paths and executing system-level commands.

`time`: Provides delay functions to prevent overly frequent detection attempts.

`speech`: The core module responsible for processing audio input and detecting the wake word.

Initializing the wonderecho Class

{lineno-start=9}

```
port = '/dev/ttyUSB0'
kws = awake.WonderEchoPro(port)
```

Attempts to Turn Off the Cooling Fan on Raspberry Pi 5

{lineno-start=13}

```
try:  # If a fan is present, it's recommended to turn it off before detection to reduce interference
    os.system('pinctrl FAN_PWM op dh')
except:
    pass
```

Purpose: Attempts to turn off the cooling fan by executing the system command `pinctrl FAN_PWM op dh`. This helps minimize background noise from the fan that could interfere with wake word detection.

Error Handling: If the command fails (e.g., due to unsupported hardware), the program catches the exception and continues running without interruption.

Main Wake Word Detection Loop

{lineno-start=18}

```
kws.start() # Start detection
print('start...')
```

The program starts the wake word detection thread using kws.start().

It prints start... to indicate that detection has been successfully initiated.

Main Program Logic

{lineno-start=20}

```
while True:
    try:
        if kws.wakeup(): # Wake-up detected
            print('hello hiwonder')
        time.sleep(0.02)
    except KeyboardInterrupt:
        kws.exit() # Cancel processing
        try:
            os.system('pinctrl FAN_PWM a0')
        except:
            pass
        break
```

During each iteration, the program checks whether the wake word has been detected. If the wake word is detected, it prints keyword detected.

The detection frequency is controlled using time.sleep(0.02) to prevent excessive CPU usage.

Pressing Ctrl+C triggers a KeyboardInterrupt, which gracefully exits the detection loop.

Upon exit, the program calls kws.exit() to stop the wake word detection process.

The fan is then restored to its original state (if applicable).

(4) Extended Functionality

Modifying the Wake-Up Response Text

In this section, you'll learn how to change the message that appears after a successful wake word detection.

① For example, if the wake word "**HELLO_HIWONDER**" is detected, and you'd like the program to print "**hello**" instead of the default message, follow the steps below. Navigate to the large_models directory and open the script with:

```
vim wakeup_demo.py
```

② Press i to enter INSERT mode (you'll see -- INSERT -- at the bottom of the screen). Locate the line '**print('hello hiwonder')**', and modify it to 'print('hello')'

```
i
```

<img  src="../_static/media/chapter_19/section_1.2/03/image17.png" style="width:500px" class="common_img"/>

③ Press ESC, then type **:wq** and press Enter to save and exit.

```
:wq
```

④ Finally, run the program with:

```
python3 wakeup_demo.py
```

(5) Creating Custom Firmware for WonderEchoPro

If you'd like to create more advanced or customized wake words and voice commands, please refer to the document titled:  

"[**Appendix →  Firmware Flashing Tool → Creating Firmware for WonderEchoPro**](https://drive.google.com/drive/folders/1Na86By9er9Jj1_1YXz3sxAwePrIgSUcN?usp=sharing)".

* **Speech Recognition**

(1) What is Speech Recognition?

Speech Recognition (Speech-to-Text, STT) is a technology that converts human speech signals into text or executable commands. In this course, we will implement speech recognition functionality using Alibaba OpenAI's Speech Recognition API.

(2) How It Works

The wave library is used to extract audio data. The extracted audio is then sent to OpenAI's ASR (Automatic Speech Recognition) model. The recognized text returned by the ASR model is stored in speech_result for use in subsequent processes.

(3) Preparation Before the Experiment

Before proceeding, refer to the course "[**19.1.1 Large Speech Model Courses -> Large Language Model Deployment**](#anchor19.1.1.2)" to obtain your API key, and make sure to add it into the configuration file (config).

(4) Experiment Steps

① Power on the device and connect to it using MobaXterm.  

(For detailed instructions, please refer to [Appendix ->Remote Connection Tools and Instructions](https://drive.google.com/drive/folders/17mfRH9lmP9OYO4_LAyzkRnHfytqRYldJ?usp=sharing).)

② Navigate to the program directory by entering the following command:

```
cd large_models/
```

③ Open the configuration file to input your API Key by entering the command below. Press i to enter INSERT mode and enter your API Key. Once finished, press Esc, type :wq, and hit Enter to save and exit.

```
vim config.py
```

<img  src="../_static/media/chapter_19/section_1.2/04/image3.png" style="width:500px" class="common_img"/>

④ Run the speech recognition program with:

```
python3 openai_asr_demo.py
```

(5) Function Realization

After the program starts, the microphone will recognize the recorded audio content from the user and print the converted text output.

<img  src="../_static/media/chapter_19/section_1.2/04/image5.png" style="width:500px" class="common_img"/>

(6) Brief Program Analysis

This program implements a speech recognition system by calling OpenAI's Speech-to-Text API to convert audio files into text.

The program source code is located at: `/home/ubuntu/large_models/openai_asr_demo.py`

① Module Import

{lineno-start=6}

```
from speech import speech
```

The speech module encapsulates ASR (Automatic Speech Recognition) functionalities, such as connecting to an external ASR service.

② Define ASR Class

{lineno-start=11}

```
asr = speech.RealTimeOpenAIASR()
```

asr = speech.RealTimeOpenAIASR()

This line creates a real-time speech recognition object named asr. The RealTimeOpenAIASR class is used to interact with the speech recognition service.

③ Speech Recognition Functionality

{lineno-start=13}

```
asr.update_session(model='whisper-1', language='en', threshold=0.2, prefix_padding_ms=300, silence_duration_ms=800) 
```

An ASR client object is created to prepare for invoking the speech recognition service.

The asr.asr() method is called to send the audio file (wav) to the ASR service for recognition.

The recognized result (typically text) is printed to the console.

(7) Function Extension

You can modify the model name to enable speech recognition in various languages, such as Chinese, English, Japanese, and Korean.

① Enter the following command to edit the script:

```
vim openai_asr_demo.py
```

② Press the i key to enter INSERT mode, and update the model setting. For example, modify it to use the gpt-4o-transcribe model.

```
i
```

<img  src="../_static/media/chapter_19/section_1.2/04/image10.png" style="width:500px" class="common_img"/>

③ Then, run the program with the command:

```
python3 openai_asr_demo.py
```

④ Record a sample sentence such as "**Hello, can you hear me clearly?**", and the recognized text will be printed on the console.

<img  src="../_static/media/chapter_19/section_1.2/04/image12.png" style="width:500px" class="common_img"/>

* **Speech Synthesis**

(1) What is Speech Synthesis?

Speech synthesis (SS) is a technology that converts written text into intelligible spoken audio. It enables computers to generate natural, human-like speech for communication or information delivery.

In this course, we will run a program that processes text using a large language model and generates corresponding audio.

(2) How It Works

The program first sends the text to the OpenAI TTS (Text-to-Speech) model. The model returns the generated audio data, which is saved as a file named tts_audio.wav for playback or storage.

(3) Preparation Before the Experiment

Refer to the course "[**Large Language Model Deployment**](#anchor19.1.1.2)" to obtain your API key, and update the configuration file accordingly.

(4) Experiment Steps

① Power on the device and connect to it using MobaXterm "(**refer to the [appendix -> Remote Connection Tools and Instructions](https://drive.google.com/drive/folders/17mfRH9lmP9OYO4_LAyzkRnHfytqRYldJ?usp=sharing) for detailed guidance**)".

② Navigate to the program directory by entering the following command:

```
cd large_models/
```

③ Open the configuration file to enter your API Key. After editing, press Esc, type :wq, and hit Enter to save and exit:

```
vim config.py
```

<img  src="../_static/media/chapter_19/section_1.2/05/image3.png" style="width:500px" class="common_img"/>

④ Finally, run the program with the following command:

```
python3 openai_tts_demo.py
```

(5) Function Realization

Upon running the program, it will play an audio message saying "**Hello, Can I Help You**", and simultaneously save the audio file with the same content to the following directory:  

`/home/ubuntu/large_models/resources/audio/`

<img  src="../_static/media/chapter_19/section_1.2/05/image5.png" style="width:500px" class="common_img"/>

(6) Brief Program Analysis

This program is a speech synthesis system based on OpenAI's Text-to-Speech (TTS) API, capable of converting text into audio files. It supports input text and outputs audio in formats like PCM, WAV, FLAC, AAC, Opus, and MP3. By specifying the desired text, the program sends the request to the API and returns the synthesized audio, which can be played or saved locally.

The source code for this program is located at:  `/home/ubuntu/large_models/openai_tts_demo.py`

① Module Import

{lineno-start=5}

```
from config import *
from speech import speech  
```

speech: This module encapsulates the TTS functionalities.

② Definition for TTS Class

{lineno-start=8}

```
tts = speech.RealTimeOpenAITTS()
tts.tts("Hello, Can I help you?") # https://platform.openai.com/docs/guides/text-to-speech
tts.tts("Hello, Can I help you?", model="tts-1", voice="onyx", speed=1.0, instructions='Speak in a cheerful and positive tone.')
tts.save_audio("Hello, Can I help you?", model="gpt-4o-mini-tts", voice="onyx", speed=1.0, instructions='Speak in a cheerful and positive tone.', audio_format='wav', save_path="./resources/audio/tts_audio.wav")
```

`speed`: Specifies the playback speed; the default value is 1.

For intelligent real-time applications, it is recommended to use the gpt-4o-mini-tts model. 

Other available models include tts-1 and tts-1-hd. tts-1 offers lower latency but with slightly reduced quality compared to tts-1-hd.

Voice Options: nova, shimmer, echo, onyx, fable, alloy, ash, sage, coral.

For more details, you can refer to the OpenAI documentation:

<https://platform.openai.com/docs/guides/text-to-speech>

③ Function Extension

To change the voice, follow these steps:

Step1 : Open the program by entering the command:

```
vim openai_tts_demo.py
```

Step2 : Press i on your keyboard to enter INSERT mode. Locate the line voice="**onyx**" and modify it to voice="**nova**".

```
i
```

<img  src="../_static/media/chapter_19/section_1.2/05/image9.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.2/05/image10.png" style="width:500px" class="common_img"/>

Step3 : Press Esc, then type :wq and hit Enter to save and exit.

```
:wq
```

<img  src="../_static/media/chapter_19/section_1.2/05/image11.png" style="width:500px" class="common_img"/>

Step4 : Execute the program with the following command:

```
python3 openai_tts_demo.py
```

<img  src="../_static/media/chapter_19/section_1.2/05/image12.png" style="width:500px" class="common_img"/>

Once the program starts, the speaker will play the synthesized audio using the newly selected voice style.

* **Voice Interaction**

(1) What is Voice Interaction?

Voice Interaction (VI) refers to a method of communication between humans and computers or devices through spoken language. It integrates speech recognition and speech synthesis, enabling devices to both understand user commands and respond naturally, creating true two-way voice communication. To achieve natural voice interaction, factors such as semantic understanding and sentiment analysis must also be considered, allowing the system to accurately interpret user intent and provide appropriate responses.

This approach can be used as the foundation for developing our own AI assistant features.

(2) How It Works

First, the wake word detection module listens for a specific wake-up word. Once detected, it initiates audio recording. After recording, Automatic Speech Recognition (ASR) converts the audio into text, which is then sent to a Large Language Model (LLM) to generate an appropriate response. The generated text is subsequently converted into speech through a Text-to-Speech (TTS) module and played back to the user. This entire process enables seamless and natural interaction between the user and the voice assistant.

(3) Experiment Steps

① Power on the device and connect to it via MobaXterm (refer to Appendix "**5.1 Remote Connection Tools and Instructions**" for connection guidance).

② To check the microphone's port number, first disconnect the microphone and run the command. Then reconnect the microphone and run the command again to determine the port number (Note: do not connect any other USB devices during this process).

```
ll /dev | grep USB
```

After disconnecting the microphone, no USB device should appear.

<img  src="../_static/media/chapter_19/section_1.2/06/image2.png" style="width:500px" class="common_img"/>

Upon reconnecting the microphone, a USB port (e.g., ttyCH341USB1) will be listed (make sure to note this device name). The device name may vary depending on the main controller.

③ Navigate to the program directory:

```
cd large_models/
```

④ Open the configuration file to enter your API Key. After editing, press Esc, then type :wq and hit Enter to save and exit:

```
vim config.py
```

<img  src="../_static/media/chapter_19/section_1.2/06/image4.png" style="width:500px" class="common_img"/>

⑤ Enter the port number you obtained and modify the corresponding microphone port settings for either WonderEcho Pro or the six-microphone setup. Uncomment the configuration for the port you intend to use and comment out the settings for any unused ports.

```
vim openai_interaciton_demo.py
```

If you are using the WonderEcho Pro, modify the corresponding section:

If you are using the 6-Microphone Array, modify the relevant section:

<img  src="../_static/media/chapter_19/section_1.2/06/image5.png" style="width:500px" class="common_img"/>

⑥ Run the program:

```
python3 openai_interaciton_demo.py
```

⑦ To stop the program at any time, simply press Ctrl+C.

(4) Function Realization

After successful execution, the voice device will announce 'I'm ready.' Then, upon hearing the wake-up word 'HELLO_HIWONDER,' the device will respond with 'I'm here,' indicating that the assistant has been successfully awakened. You can now ask the AI assistant any questions:

For example: 'What are some fun places to visit in New York?'

<img  src="../_static/media/chapter_19/section_1.2/06/image7.png" style="width:500px" class="common_img"/>

(5) Brief Program Analysis

The program integrates voice recognition, speech synthesis, and intelligent response functionalities to create a voice assistant. Interaction is initiated through the wake-up word (HELLO_HIWONDER). Users can converse with the assistant via voice commands, and the assistant will respond using text-to-speech technology. The overall structure is clear, with distinct modules that are easy to expand and maintain.

The source code for this program is located at: `/home/ubuntu/large_models/openai_interaction_demo.py`

(1) Module Import

{lineno-start=5}

```
import os
import time
from config import *
from speech import awake
from speech import speech
```

time: Used to control the interval between program executions.

speech: The core module, integrating wake-up word detection, speech activity detection, speech recognition, TTS, and LLM.

(2) Definition of Audio File Paths

{lineno-start=11}

```
wakeup_audio_path = './resources/audio/en/wakeup.wav'
start_audio_path = './resources/audio/en/start_audio.wav'
no_voice_audio_path = './resources/audio/en/no_voice.wav'
```

This section configures the audio file paths used by various functional modules, such as wake-up sounds, recording storage paths, and prompt sounds.

The text-to-speech (TTS) module is initialized to convert LLM responses into speech.

(3) Main Functional Logic

{lineno-start=33}

```
def main():
    kws.start()
    while True:
        try:
            if kws.wakeup(): # Wake word detected
                speech.play_audio(wakeup_audio_path)  # Play wake-up sound
                asr_result = asr.asr() # Start voice recognition
                print('asr_result:', asr_result)
                if asr_result:
                    # Send the recognition result to the agent for a response
                    response = client.llm(asr_result, model='gpt-4o-mini')
                    print('llm response:', response)
                    tts.tts(response)
                else:
                    speech.play_audio(no_voice_audio_path)
            time.sleep(0.02)
        except KeyboardInterrupt:
            kws.exit() 
            try:
                os.system('pinctrl FAN_PWM a0')
            except:
                pass
            break
        except BaseException as e:
            print(e)
```

Wake-up Detection: Continuously monitors for the wake-up word. Once detected, it stops the wake-up detection and plays the wake-up prompt sound.

Voice Processing: Records and recognizes the user's speech, uses the language model to generate a response, and then converts the response into speech for playback.

Error Handling: Catches exit signals and runtime errors to ensure the program exits safely and releases resources.

### 19.1.3 Vision Language Model Courses

* **Overview of Vision Language Model**

Vision Language Model (VLM) integrate visual recognition capabilities into traditional Language Model (LLM), enabling more powerful interactions between vision and language through multimodal inputs.

(1) Basic Concept

Vision Language Model (VLM) are a type of artificial intelligence model that leverages deep learning techniques to learn from and process large-scale visual data. These models often adopt convolutional neural network (CNN) architectures, enabling them to extract rich visual features from images or video streams and perform various tasks such as image classification, object detection, and facial recognition. In theory, VLM possess powerful capabilities in feature extraction and pattern recognition, making them widely applicable in fields like autonomous driving, facial recognition, and medical imaging analysis.

(2) Features

**Multimodal Input and Output**: VLM can process both images and text as input and generate various forms of output, including text, images, charts, and more.

**Powerful Visual-Semantic Understanding**: With extensive knowledge accumulated from large-scale visual datasets, VLMsexcel at tasks such as object detection, classification, and image captioning.

**Visual Question Answering (VQA):** VLM can engage in natural language conversations based on the content of input images, accurately answering vision-related questions.

**Image Generation:** Some advanced VLM are capable of generating simple image content based on given conditions.

**Deep Visual Understanding:** These models can recognize intricate details within images and explain underlying logical and causal relationships.

**Cross-Modal Reasoning:** VLM can leverage visual and linguistic information together, enabling reasoning across modalities, such as inferring from language to vision and vice versa.

**Unified Visual and Language Representation Space:** By applying attention mechanisms, VLM establish deep connections between visual and semantic information, achieving unified multimodal representations.

**Open Knowledge Integration:** VLM can integrate both structured and unstructured knowledge, enhancing their understanding of image content.

(3) How It Works

The working principle of Vision Language Model is primarily based on deep learning techniques, particularly Convolutional Neural Networks (CNNs) and Transformer architectures. Through multiple layers of neurons, these models perform feature extraction and information processing, enabling them to automatically recognize and understand complex patterns within images.

In a VLM, the input image first passes through several convolutional layers, where local features such as edges, textures, and shapes are extracted. Each convolutional layer is typically followed by an activation function (e.g., ReLU) to introduce non-linearity, allowing the model to learn more complex representations. Pooling layers are often used to reduce the dimensionality of the data while preserving important information, helping to optimize computational efficiency.

As the network deepens, it gradually transitions from extracting low-level features (like edges and corners) to higher-level features (such as objects and scenes). For classification tasks, the final feature vector is passed through fully connected layers to predict the probability of different target categories. For tasks like object detection and segmentation, the model outputs bounding boxes or masks to indicate the location and shape of objects within the image.

Transformer-based VLM divide images into small patches, treating them as sequential data, and apply self-attention mechanisms to capture global relationships within the image. This approach is particularly effective at modeling long-range dependencies, enabling VLM to excel at understanding complex visual scenes.

Training VLM typically requires large-scale labeled datasets. Through backpropagation, the model optimizes its parameters to minimize the loss between predictions and ground-truth labels. Pretraining on massive datasets allows the model to acquire general-purpose visual features, while fine-tuning on specific tasks further improves performance for specialized applications.

Thanks to this design, Visual Language Models are able to process and understand visual data effectively, and are widely used in applications like image classification, object detection, and image segmentation, driving rapid progress in the field of computer vision.

(4) Application Scenarios

① Image Captioning

VLM can automatically generate textual descriptions based on input images. This capability is particularly valuable for social media platforms, e-commerce websites, and accessibility technologies, such as providing visual content descriptions for visually impaired users.

② Visual Question Answering

Users can ask questions related to an image, such as "**What is in this picture?**" or "**What color is the car?**" The model analyzes the image content and provides accurate, natural-language responses, making it highly applicable in fields such as education, customer support, and information services.

③ Image Retrieval

In image search engines, users can perform searches using text descriptions, and Vision Language Model (VLM) can understand the descriptions and return relevant images. This capability is especially important on e-commerce platforms, where it allows users to find desired products more intuitively.

④ Augmented Reality (AR)

Vision Language Model (VLM) can provide real-time visual feedback and language-based explanations in augmented reality applications. When users view real-world scenes through a device's camera, the system can overlay relevant information or guidance, enhancing the overall user experience.

⑤ Content Creation and Editing

In design and creative tools, Vision Language Model (VLM) can generate relevant text content or suggestions based on a user's visual input (such as sketches or images), helping users complete creative work more efficiently.

⑥ Social Media Interaction

On social media platforms, VLM can generate appropriate comments or tags based on user-uploaded images, enhancing engagement and interaction.

⑦ Medical Imaging Analysis

In the healthcare field, VLM can be used to analyze medical images (such as X-rays and CT scans) and generate diagnostic reports or recommendations, assisting doctors in making more accurate decisions.

<p id="anchor19.1.3.2"></p>

* **Vision Language Model Accessing**

> [!NOTE]
>
> * **This section requires the configuration of the API key in "[Vision Language Model Accessing](#anchor19.1.3.2)" before proceeding. Additionally, ensure that the images to be used in this section are imported.**
> * **This experiment requires either an Ethernet cable or Wi-Fi connection to ensure the main control device can access the network properly.**

(1) Experiment Steps

Execute the following command to navigate to the directory of Large Model.

```
cd large_models/
```

Run the program:

```
python3 openai_vllm_understand.py
```

(2) Function Realization

After running the program, the output printed matches our request of "**Describe the image**".

<img  src="../_static/media/chapter_19/section_1.3/02/image4.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/02/image5.jpeg" style="width:300px" class="common_img"/>

* **Vision Language Model: Object Detection**

> [!NOTE]
>
> * **This section requires the configuration of the API key in "[19.1.3 Vision Language Module Courses -> Vision Language Model Accessing](#anchor19.1.3.2)" before proceeding. Additionally, ensure that the images to be used in this section are imported.**
> * **This experiment requires either an Ethernet cable or Wi-Fi connection to ensure the main control device can access the network properly.**
> * **In this course, we will use a program to transmit an image to the large model for recognition, which will then identify and locate the objects within the image by drawing bounding boxes around them.**

(1) Experiment Steps

① Execute the following command to navigate to the directory of Large Model.

```
cd large_models/
```

② Run the program:

```
python3 qwen_vllm_detect_demo.py
```

(2) Function Realization

After running the program, the positions of the fruits in the image will be circled.

<img  src="../_static/media/chapter_19/section_1.3/03/image4.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/03/image5.png" style="width:400px" class="common_img"/>

(3) Function Expansion

We can switch the image and change the large model to experience different functionalities of various models.

Change Pictures:

① Click on the path box to navigate to the following directory: `/home/ubuntu/large_models/resources/pictures/`

Here, you can drag in other images, for example, in the apples.png format.

<img  src="../_static/media/chapter_19/section_1.3/03/image6.png" style="width:400px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/03/image7.png" style="width:400px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/03/image8.png" style="width:400px" class="common_img"/>

② Then, input the command:

```
vim large_models/qwen_vllm_detect_demo.py
```

③ Press the "**i**" key on your keyboard, which will display **"INSERT"** at the bottom.

```
i
```

<img  src="../_static/media/chapter_19/section_1.3/03/image10.png" style="width:500px" class="common_img"/>

④ Change the image recognition path from: `./resources/pictures/test_image_understand.jpeg`

To: image = cv2.imread('./resources/pictures/apples.png')

<img  src="../_static/media/chapter_19/section_1.3/03/image11.png" style="width:500px" class="common_img"/>

⑤ Next, input the following command and execute the program again to see the results

```
python3 qwen_vllm_detect_demo.py
```

<img  src="../_static/media/chapter_19/section_1.3/03/image12.png" style="width:400px" class="common_img"/>

* **Vision Language Model: Scene Understanding**

> [!NOTE]
>
> * This section requires the configuration of the API key in "[**Vision Language Model Accessing**](#anchor19.1.3.2)" before proceeding. Additionally, ensure that the images to be used in this section are imported.
>
> * This experiment requires either an Ethernet cable or Wi-Fi connection to ensure the main control device can access the network properly.
>
> * In this course, we will use a program to send an image to the large model for recognition and generate a description of the content within the image.

(1) Experiment Steps

① Execute the following command to navigate to the directory of Large Model.

```
cd large_models/
```

② Run the program:

```
python3 openai_vllm_understand.py
```

(2) Function Realization

After running the program, the output printed matches our request of "**Describe the image**".

<img  src="../_static/media/chapter_19/section_1.3/04/image4.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/04/image5.jpeg" style="width:400px" class="common_img"/>

(3) Function Expansion

If you need to recognize your own image, you should place the image in the corresponding path and modify the image path in the program.

① First, drag your image directly into the ~/large_models/resources/pictures/ path using MobaXterm, and rename the image to test.png.

<img  src="../_static/media/chapter_19/section_1.3/04/image6.png" style="width:500px" class="common_img"/>

② Then, open the scene understanding script by entering the following command in the terminal:

```
vim ~/large_models/vllm_understand.py
```

③ Change the image path in the code to reflect the name of your image (e.g., test.png).

<img  src="../_static/media/chapter_19/section_1.3/04/image8.png" style="width:500px" class="common_img"/>

④ Run the program:

```
python3 ~/large_models/openai_vllm_understand.py
```

<img  src="../_static/media/chapter_19/section_1.3/04/image10.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/04/image11.png" style="width:500px" class="common_img"/>

* **Vision Language Model: Optical Character Recognition** 

> [!NOTE]
>
> * This section requires the configuration of the API key in "**[ Vision Language Model Accessing](#anchor19.1.3.2)**" before proceeding. Additionally, ensure that the images to be used in this section are imported.
>
> * This experiment requires either an Ethernet cable or Wi-Fi connection to ensure the main control device can access the network properly.
>
> * In this course, we use a program to transmit an image to the large model for recognition, extracting and identifying the text within the image.

(1) Experiment Steps

① Execute the following command to navigate to the directory of Large Model.

```
cd large_models/
```

② Run the program:

```
python3 openai_vllm_ocr.py
```

(2) Function Realization

After running the program, the output printed will be consistent with the content of the image sent.

<img  src="../_static/media/chapter_19/section_1.3/05/image4.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/05/image5.png" style="width:400px" class="common_img"/>

(3) Function Expansion

We can switch the image and change the large model to experience different functionalities of various models.

Change Pictures：

① Drag the image directly into the `~/large_models/resources/pictures/` path using MobaXterm. Here, we can drag in the image named 'ocr1.png' as an example, and let the program recognize the text 'COME ON'.

<img  src="../_static/media/chapter_19/section_1.3/05/image6.png" style="width:500px" class="common_img"/>

<img  src="../_static/media/chapter_19/section_1.3/05/image7.png" style="width:400px" class="common_img"/>

② Then, input the command:

```
vim ~/large_models/openai_vllm_ocr.py
```

③ Press the **"i"** key on your keyboard, which will display **"INSERT"** at the bottom.

```
i
```

<img  src="../_static/media/chapter_19/section_1.3/05/image9.png" style="width:500px" class="common_img"/>

④ Change the image recognition path from: ./resources/pictures/ocr.jpeg

To: image = cv2.imread('./resources/pictures/ocr1.png')

```
image = cv2.imread('./resources/pictures/ocr1.png)
```

⑤ Run the program:

```
python3 ~/large_models/openai_vllm_ocr.py
```

<img  src="../_static/media/chapter_19/section_1.3/05/image12.png" style="width:500px" class="common_img"/>

### 19.1.4 Multimodal Model Basic Courses

* **Overview of Multimodal Model**

The emergence of Multimodal Model is built upon continuous advancements in the fields of Large Language Model (LLM) and Vision Language Model (VLM).

(1) Basic Concept

As LLM continue to improve in language understanding and reasoning capabilities, techniques such as instruction tuning, in-context learning, and chain-of-thought prompting have become increasingly widespread. However, despite their strong performance on language tasks, LLM still exhibit notable limitations in perceiving and understanding visual information such as images. At the same time, VLM have made significant strides in visual tasks such as image segmentation and object detection, and can now be guided by language instructions to perform these tasks, though their reasoning abilities still require further enhancement.

(2) Features

The core strength of Multimodal Model lies in their ability to understand and manipulate visual content through language instructions. Through pretraining and fine-tuning, these models learn the associations between different modalities—such as how to generate descriptions from images or how to identify and classify objects in visual data. Leveraging self-attention mechanisms from deep learning, Multimodal Model can effectively capture relationships across modalities, allowing them to synthesize information from multiple sources during reasoning and decision-making processes.

**Multimodal Fusion Capability:** Multimodal Model can process and understand multiple types of data simultaneously, including text, images, and audio. This fusion ability enables the models to build connections across modalities, leading to a more comprehensive understanding of information. For instance, a model can generate natural language descriptions based on an image or identify specific objects within an image based on a text query.

**Enhanced Contextual Understanding:** By integrating information from different modalities, Multimodal Model excel at contextual understanding. They can not only recognize content within a single modality but also combine clues from multiple sources to make more accurate judgments and decisions in complex tasks.

**Flexible Interaction Methods:** Users can interact with Multimodal Model through natural language instructions, making communication with the models more intuitive without requiring knowledge of complex programming or operations. For example, users can simply ask about details in an image, and the model can provide relevant answers.

**Scalability:** The architecture and training methods of Multimodal Model allow them to adapt to new modalities and tasks. As technology evolves, additional types of data—such as videos or sensor readings—can be incorporated, expanding their range of applications and capabilities.

**Strong Generative Capabilities:** Similar to large language models, Multimodal Model perform exceptionally well in generating both textual and visual content. They can produce natural language descriptions, summaries, and even create novel visual outputs, meeting a wide variety of application needs.

**Improved Reasoning Abilities:** Although challenges remain, Multimodal Model demonstrate significantly enhanced reasoning capabilities compared to traditional single-modality models. By integrating multimodal information, they can reason effectively in more complex scenarios, supporting advanced tasks such as logical reasoning and sentiment analysis.

**Adaptability and Personalization:** Multimodal Model can be fine-tuned to meet user-specific needs and preferences, enabling highly personalized services. This adaptability offers great potential for applications in fields such as education, entertainment, and customer service.

(3) How It Works

The working principle of Multimodal Model is built upon advanced deep learning and neural network technologies, with a core focus on fusing data from different modalities to understand and tackle complex tasks. At the foundation, Multimodal Model often adopt architectures similar to Transformers, which are highly effective at capturing relationships between different parts of input data. During training, these models are exposed to massive amounts of multimodal data—such as images, text, and audio—and leverage large-scale unsupervised learning for pretraining. Through this process, the models learn the commonalities and differences across modalities, enabling them to grasp the intrinsic connections between various types of information.

In practice, incoming text and visual data are first embedded into a shared representation space. Text inputs are transformed into vectors using word embedding techniques, while images are processed through methods like Convolutional Neural Networks (CNNs) to extract visual features. These vectors are then fed into the model's encoder, where self-attention mechanisms analyze the relationships across modalities, identifying and focusing on the most relevant information.

After encoding, the model generates a multimodal contextual representation that blends both the semantic information of the text and the visual features of the image. When a user provides a natural language instruction, the MLLM parses the input and interprets the intent by leveraging the contextual representation. This process involves reasoning and generation capabilities, allowing the model to produce appropriate responses based on its learned knowledge, or to perform specific actions in visual tasks.

Finally, the Multimodal Model's decoder translates the processed information into outputs that users can easily understand—such as generating textual descriptions or executing targeted visual operations. Throughout this process, the emphasis is on the fusion and interaction of information across different modalities, enabling Multimodal Model to excel at handling complex combinations of natural language and visual content. This integrated working mechanism empowers Multimodal Model with powerful functionality and flexibility across a wide range of application scenarios.

(4) Application Scenarios

① Education

Multimodal Model can be used to create personalized learning experiences. By combining text and visual content, the model can provide students with rich learning materials—for example, explaining scientific concepts through a mix of images and text to enhance understanding. Additionally, in online courses, the model can dynamically adjust content based on the learner's performance, offering customized learning suggestions in real time.

② Healthcare

Multimodal Model can assist doctors in diagnosis and treatment decisions. By analyzing medical images (such as X-rays or MRIs) alongside relevant medical literature, the model helps doctors access information more quickly and provides evidence-based recommendations. This application improves diagnostic accuracy and efficiency.

③ Entertainment

Multimodal Model can be used for content generation, such as automatically creating stories, scripts, or in-game dialogues. By incorporating visual elements, the model can provide rich scene descriptions for game developers, enhancing immersion. Additionally, on social media platforms, Multimodal Model can analyze user-generated images and text to help recommend suitable content.

④ Advertising and Marketing

Multimodal Model can analyze consumer behavior and preferences to generate personalized advertising content. By combining text and images, ads can better capture the attention of target audiences and improve conversion rates.

Finally, Multimodal Model also play a role in scientific research. By processing large volumes of literature and image data, the model can help researchers identify trends, generate hypotheses, or summarize findings, accelerating scientific discovery.

* **Agent Behavior Orchestration**

> [!NOTE]
>
> * This section requires the configuration of the API key in "[**Vision Language Model Accessing**](#anchor19.1.3.2)" before proceeding. Additionally, ensure that the images to be used in this section are imported.
>
> * This experiment requires either an Ethernet cable or Wi-Fi connection to ensure the main control device can access the network properly.
>
> * The purpose of this course experiment is to obtain data in a specified format returned by the large model based on the prompt words set in the model. During development, you can use the returned data for further tasks.

(1) Experiment Steps

① To check the microphone's port number, first disconnect the microphone and run the command. Then reconnect the microphone and run the command again to determine the port number (Note: do not connect any other USB devices during this process).

```
ll /dev | grep USB
```

After disconnecting the microphone, no USB device should appear.

<img  src="../_static/media/chapter_19/section_1.4/image2.png" style="width:500px" class="common_img"/>

Upon reconnecting the microphone, a USB port (e.g., ttyCH341USB1) will be listed (make sure to note this device name). The device name may vary depending on the main controller.

② Execute the following command to navigate to the directory of Large Model.

```
cd large_models/
```

③ Open the configuration file to enter your API Key. After editing, press Esc, then type :wq and hit Enter to save and exit:

```
vim config.py
```

<img  src="../_static/media/chapter_19/section_1.4/image4.png" style="width:500px" class="common_img"/>

④ Fill in the detected port number and update the corresponding microphone port settings for either the WonderEcho Pro or the Six-channel Microphone.  

Uncomment the port you wish to use and comment out the settings for any unused ports.

```
vim openai_agent_demo.py
```

Modify the settings as follows. For WonderEcho Pro, update the corresponding configuration

<img  src="../_static/media/chapter_19/section_1.4/image5.png" style="width:500px" class="common_img"/>

For 6-channel Microphone, update the respective settings:

<img  src="../_static/media/chapter_19/section_1.4/image6.png" style="width:500px" class="common_img"/>

⑤ Run the program:

```
python3 openai_agent_demo.py
```

⑥ The program will print the prompts configured for the large model. The large model will then return data formatted according to these prompts.

<img  src="../_static/media/chapter_19/section_1.4/image8.png" style="width:500px" class="common_img"/>



(2) Function Realization

① After running the program, the voice device will announce, **"I'm ready".** At this point, say **"HELLO_HIWONDER"** to the device to activate the agent.  

When the device responds with "**I'm here**", it indicates that the agent has been successfully awakened. To modify the wake word. For the Six-channel Microphone, refer to Section 2.3 Voice Wake-Up – 2. 6-Microphone Circular Array for instructions on customizing the wake word. For WonderEcho Pro, refer to Section "[**Firmware Flashing Tool->WonderEchoPro Firmware Generation**](https://drive.google.com/drive/folders/1Na86By9er9Jj1_1YXz3sxAwePrIgSUcN?usp=sharing)".

② After updating the wake word, you can say: "Take two steps forward, turn left and take one step back". The agent will respond according to the format we have defined.

<img  src="../_static/media/chapter_19/section_1.4/image9.png" style="width:500px" class="common_img"/>



## 19.2 Multimodal Large Model Applications

<p id ="anther19.2.1"></p>

### 19.2.1 Large Model API Key Setup

> [!NOTE]
>
> **This section requires registering on the official OpenAI website and obtaining an API key for accessing large language models.**

#### 19.2.1.1 OpenAI Account Registration and Deployment

1) Copy and open the following URL: https://platform.openai.com/docs/overview, then click the **Sign Up** button in the upper-right corner.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image1.png"  />

2) Register and log in using a Google, Microsoft, or Apple account, as prompted.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image2.png" style="width:500px" />

3) After logging in, click the **Settings** button, then go to **Billing**, and click **Payment Methods** to add a payment method. The payment is used to purchase **tokens**.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image3.png"  />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image4.png" style="width:700px" />

4) After completing the preparation steps, click **API Keys** and create a new key. Follow the prompts to fill in the required information, then save the key for later use.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image5.png"  />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image6.png"  />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image7.png"  />

5) The creation and deployment of the large model have been completed, and this API will be used in the following sections.

#### 19.2.1.2 OpenRouter Account Registration and Deployment

1) Copy the URL https://openrouter.ai/ into a browser and open the webpage. Click **Sign in** and register or sign in using a Google account or another available account.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image8.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image9.png"  />

2) After logging in, click the icon in the top-right corner, then select **Credits** to add a payment method.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image10.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image11.png" style="width:700px" />

3) Create an API key. Go to **API Keys**, then click **Create Key**. Follow the prompts to generate a key. Save the API key securely for later use.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image12.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image13.png" style="width:700px" />

The creation and deployment of the large model have been completed, and this API will be used in the following sections.

#### 19.2.1.3 API Configuration

1. Click <img src="../_static/media/chapter_19/section_3/media/image14.png"   /> to open a terminal and enter the following command to open the configuration file. Press the **i** key to enter input mode.

```bash
vim /home/ubuntu/ros2_ws/src/large_models/large_models/large_models/config.py
```

2. Fill in the obtained Large Model API Key in the corresponding parameter, as shown in the red box in the figure below.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image15.png"  />

3. Press **Esc**, then enter the command and press **Enter** to save and exit the configuration file.

```bash
:wq
```

<p id ="anther19.2.2"></p>

### 19.2.2 Version Confirmation

Before starting features, verify that the correct microphone configuration is set in the system.

1. After remotely logging in via NoMachine, click the desktop icon <img src="../_static/media/chapter_19/section_3/media/image16.png"  /> to access the configuration interface.

2. Select the appropriate microphone version configuration according to the hardware.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image17.png"  />

3. If using the AI Voice Interaction Box, select **WonderEcho Pro** as the microphone type, as shown in the figure below.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image18.png"  style="width:300px" />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image19.png"  />

4. For the Six-Microphone Array, select **xf** as the microphone type as shown in the figure.

   <img class="common_img" src="../_static/media/chapter_19/section_3/media/image20.png"  style="width:300px" />

   <img class="common_img" src="../_static/media/chapter_19/section_3/media/image21.png"  />

5. Click **Save**.

   <img class="common_img" src="../_static/media/chapter_19/section_3/media/image22.png"  />

6. After the **Save Success** notification appears, click **Apply**.

   <img class="common_img" src="../_static/media/chapter_19/section_3/media/image23.png"  />

7. Finally, click **Quit** to exit the software interface.

   <img class="common_img" src="../_static/media/chapter_19/section_3/media/image24.png"  />


### 19.2.3 Voice Control

#### 19.2.3.1 Program Overview

Once the program starts, the voice device will announce **I’m ready**. Say the designated wake word **Hello Hiwonder** to activate the voice device, which will respond with **I’m here**. Then, the robot can be controlled via voice commands to perform corresponding actions, for example: **Move forward, backward**. The voice device will respond after processing the command and execute the corresponding action.

#### 19.2.3.2 Preparation

* **Version Confirmation**

Before starting this feature, ensure that the microphone version configuration in the system is correct. For more details, refer to section [19.2.2 Version Confirmation](#anther19.2.2).

* **Configure Large Model API-KEY**

Refer to the section [19.2.1 Large Model API Key Setup](#anther19.2.1) to set up the large model key.

#### 19.2.3.3 Operation Steps

> [!NOTE]
>
> * **Command input is case-sensitive and space-sensitive.**
>
> * **The robot must be connected to the Internet, either in STA (LAN) mode or AP (direct connection) mode via Ethernet.**

1. Open the command line terminal from the left side of the system interface.<img src="../_static/media/chapter_19/section_3/media/image25.png"  /> In the terminal window, enter the following command and press **Enter** to stop the auto-start service.

```bash
sudo systemctl stop start_app_node.service
```

2) Open the command line terminal <img src="../_static/media/chapter_19/section_3/media/image14.png"   /> from the left side of the system interface. In the terminal window, enter the following command and press **Enter** to launch the voice control feature.

```bash
ros2 launch large_models_examples llm_control_move.launch.py
```

3) When the command line displays the output shown below and announces **I’m ready**, the voice device has completed initialization. At this point, say the wake word **Hello Hiwonder**.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image26.png" style="width:700px" />

4) When the command line shows the output below and the voice device announces **I’m here**, the voice device has been successfully activated. The system will begin recording voice commands.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image27.png" style="width:700px" />

5) When the terminal displays the next output as the reference image, it shows the recognized speech transcribed by the device.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image28.png" style="width:700px" />

6. When the command line displays the output shown below, the cloud-based large language model has been successfully invoked. It processes the user’s command, provides a verbal response, and generates actions corresponding to the semantic meaning of the command.

   The response is automatically generated by the large model. Only the semantic accuracy of the reply is guaranteed, while the wording and formatting may vary.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image28.png" style="width:700px" />

7) When the command line displays the output shown below, the current dialogue session has ended. Refer to Step 4 to reactivate the voice device with the wake word and start a new dialogue session.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image29.png" style="width:700px" />

8) To exit the feature, press **Ctrl+C** in the terminal. If the feature does not shut down immediately, press **Ctrl+C** multiple times. If exiting fails, open a new terminal and enter the command to terminate all currently running ROS nodes and related processes.

```bash
~/.stop_ros.sh
```

#### 19.2.3.4 Program Outcome

Once the feature is activated, commands can be freely issued, for example: **Move forward, backward**. The robot will then move forward and backward accordingly.

#### 19.2.3.5 Program Analysis

* **Launch File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/llm_control_move.launch.py**

Define `launch_setup` Function

```python
def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    controller_package_path = get_package_share_directory('controller')
    large_models_package_path = get_package_share_directory('large_models')

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )


    llm_control_move_node = Node(
        package='large_models_examples',
        executable='llm_control_move',
        output='screen',
    )

    return [mode_arg,
            controller_launch,
            large_models_launch,
            llm_control_move_node,
            ]
```

This function is used to configure and launch actions in a Launch file.

`mode = LaunchConfiguration('mode', default=1)`: Defines a Launch parameter named mode with a default value of 1.

`mode_arg = DeclareLaunchArgument('mode', default_value=mode)`: Declares the mode parameter and adds it to the Launch description.

`controller_path和large_models_package_path`: Paths to the shared directories of the `controller` and `large_models` packages, which control the robot’s movement.

`controller_launch`: Uses `IncludeLaunchDescription` to include the `controller.launch.py ` file from the `controller` package.

`large_models_launch`: Uses `IncludeLaunchDescription` to include the `start.launch.py` file from the `large_models` package and passes the mode parameter.

Finally, returns a list containing all Launch actions.

* **Python File Analysis**

The program file is located at: **ros2_ws/src/large_models_examples/large_models_examples/llm_control_move.py**.

1\. Define Prompt Template (PROMPT)

```python
else:
    PROMPT = '''
**Role
You are an intelligent car that can be controlled via linear velocity on the x and y axes (in meters per second), and angular velocity on the z axis (in radians per second). The movement duration is controlled by t (in seconds).
Your job is to generate a corresponding instruction based on user input.

**Requirements
- Ensure valid velocity ranges:
Linear velocity: x, y ∈ [-1.0, 1.0] (negative values mean reverse direction)
Angular velocity: z ∈ [-1.0, 1.0] (negative: clockwise, positive: counterclockwise)
- Execute multiple actions sequentially, returning a list of movement instructions under the action field.
- Always append a stop command [0.0, 0.0, 0.0, 0.0] at the end to ensure the car halts.
- Default values:
x and y: 0.2
z: 1.0
t: 2.0
- For each action sequence, craft a short (5–10 characters), witty, and endlessly variable response to make interactions fun and engaging.
- Output only the final JSON result. No explanations, no extra output.
- Format:
{
  "action": [[x1, y1, z1, t1], [x2, y2, z2, t2], ..., [0.0, 0.0, 0.0, 0.0]],
  "response": "short response"
}
- Possess strong mathematical reasoning to interpret and compute physical quantities like distance, time, and velocity.

## Special Notes
The "action" key should contain an array of stringified movement instructions in execution order. If no valid command is found, output an empty array [].
The "response" key should contain a creatively written, concise reply that matches the required tone and length.

**Examples
Input: Move forward for 2 seconds, then rotate clockwise for 1 second
Output:
{
  "action": [[0.2, 0.0, 0.0, 2.0], [0.0, 0.0, 1.0, 1.0], [0.0, 0.0, 0.0, 0.0]],
  "response": "Full speed ahead, spin and go!"
}

Input: Move forward 1 meter
Output:
{
  "action": [[0.2, 0.0, 0.0, 5.0], [0.0, 0.0, 0.0, 0.0]],
  "response": "Got it!"
}
    '''
```

2\. Class Initialization

```python
class LLMControlMove(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.llm_result = ''
        self.running = True
        self.interrupt = False
        self.action_finish = False
        self.play_audio_finish = False
        
        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

Set state variables such as action list `action`, interrupt flag `interrupt`, and action completion flag `action_finish`. Create publishers such as TTS text publisher and subscribers such as large model results, TTS playback completion signal, and wake-up signal. Create service clients, such as for configuring the large model or enabling voice for communication and control.

3\. Set LLM Model

```python
def init_process(self):
    self.timer.cancel()

    msg = SetModel.Request()
    # msg.model = 'qwen-plus-latest'
    msg.model = llm_model
    msg.model_type = 'llm'
    msg.api_key = api_key 
    msg.base_url = base_url
    self.send_request(self.set_model_client, msg)

    msg = SetString.Request()
    msg.data = PROMPT
    self.send_request(self.set_prompt_client, msg)

    speech.play_audio(start_audio_path) 
    threading.Thread(target=self.process, daemon=True).start()
    self.create_service(Empty, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)
```

4\. `play_audio_finish_callback` Method

```python
def play_audio_finish_callback(self, msg):
  msg = SetBool.Request()
  msg.data = True
  self.send_request(self.awake_client, msg)
  # msg = SetInt32.Request()
  # msg.data = 1
  # self.send_request(self.set_mode_client, msg)
  self.play_audio_finish = msg.data
```

The method handles the callback after audio playback and re-enables the voice wake-up functionality.

5\. `process` Method

```python
def process(self):
    while self.running:
        if self.llm_result:
            msg = String()
            if 'action' in self.llm_result:  # Extract the line center using weighted averaging
                result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}') + 1])
                self.get_logger().info(str(result))
                action_list = []
                if 'action' in result:
                    action_list = result['action']
                if 'response' in result:
                    response = result['response']
                msg.data = response
                self.tts_text_pub.publish(msg)
                for i in action_list:
                    msg = Twist()
                    msg.linear.x = float(i[0])
                    msg.linear.y = float(i[1])
                    msg.angular.z = float(i[2])
                    self.mecanum_pub.publish(msg)
                    time.sleep(i[3])
                    if self.interrupt:
                        self.interrupt = False
                        self.mecanum_pub.publish(Twist())
                        break
            else:  # Otherwise, return without further handling
                response = self.llm_result
                msg.data = response
                self.tts_text_pub.publish(msg)
            self.action_finish = True 
            self.llm_result = ''
        else:
            time.sleep(0.01)
        if self.play_audio_finish and self.action_finish:
            self.play_audio_finish = False
            self.action_finish = False
            # msg = SetInt32.Request()
            # msg.data = 2
            # self.send_request(self.set_mode_client, msg)
    rclpy.shutdown()
```

The main loop handles instructions from the LLM, parses commands, executes the corresponding actions, and provides voice feedback.

### 19.2.4 Autonomous Patrolling

#### 19.2.4.1 Program Overview

When the program starts, the voice device will announce **I'm ready**.

To activate the voice device, say the designated wake word **Hello Hiwonder**. Upon successful activation, the voice device will respond with **I’m here**. Once activated, voice commands can be issued, for example, **Follow the black line and stop when you encounter an obstacle.** The terminal displays the recognized command, and the voice device provides a processed response. The robot then follows the black line detected by its camera and stops automatically when an obstacle is detected.

#### 19.2.4.2 Preparation

* **Version Confirmation**

Before starting this feature, ensure that the microphone version configuration in the system is correct. For more details, refer to section [19.2.2 Version Confirmation](#anther19.2.2).

* **Configure Large Model API-KEY**

Refer to the section [19.2.1 Large Model API Key Setup](#anther19.2.1) to set up the large model key.

#### 19.2.4.3 Operation Steps

> [!NOTE]
>
> * **Command input is case-sensitive and space-sensitive.**
>
> * **The robot must be connected to the Internet, either in STA (LAN) mode or AP (direct connection) mode via Ethernet.**

1) Click the terminal <img src="../_static/media/chapter_19/section_3/media/image25.png"  /> on the left side of the system interface to open the command line. Enter the command and press **Enter** to disable the app auto-start service.

```bash
sudo systemctl stop start_app_node.service
```

2) Open the command line terminal <img src="../_static/media/chapter_19/section_3/media/image14.png"   /> from the left side of the system interface. In the terminal window, enter the following command and press **Enter** to launch the line following feature.

```bash
ros2 launch large_models_examples llm_visual_patrol.launch.py
```

3) When the command line displays the output shown below and announces **I’m ready**, the voice device has completed initialization. At this point, say the wake word **Hello Hiwonder**.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image30.png" style="width:700px" />

4) When the command line shows the output below and the voice device announces **I’m here**, the voice device has been successfully activated. The system will begin recording voice commands.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image31.png" style="width:700px" />

5) When the terminal prints the output shown below, it indicates that the voice device has transcribed the spoken command. In this example, the command spoken is **Follow the black line and stop when you encounter an obstacle**.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image32.png" style="width:700px" />

6. When the command line displays the output shown below, the cloud-based large language model has been successfully invoked. It processes the user’s command, provides a verbal response, and generates actions corresponding to the semantic meaning of the command.

   The response is automatically generated by the large model. Only the semantic accuracy of the reply is guaranteed, while the wording and formatting may vary.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image33.png" style="width:500px" />

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image34.png" style="width:700px" />

7) To stop the feature, refer to Step 4 to reactivate the voice device again with the wake word to begin a new interaction cycle.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image35.png" style="width:700px" />

8) To exit the feature, press **Ctrl+C** in the terminal. If the feature does not exit immediately, press **Ctrl+C** multiple times.

#### 19.2.4.4 Program Outcome

Once the feature is started, voice commands can be issued to the robot. For instance, **Follow the black line and stop when you encounter an obstacle**. The robot uses its camera to detect and follow the black line, and it will stop when an obstacle is detected in its path. The system is pre-configured to recognize four line colors: red, blue, green, and black.

#### 19.2.4.5 Program Analysis

* **Launch File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/llm_visual_patrol.launch.py**.

Define `launch_setup` Function

```python
def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    app_package_path = get_package_share_directory('app')
    large_models_package_path = get_package_share_directory('large_models')

    line_following_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/line_following_node.launch.py')),
        launch_arguments={
            'debug': 'true',
        }.items(),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    llm_visual_patrol_node = Node(
        package='large_models_examples',
        executable='llm_visual_patrol',
        output='screen',
    )

    return [mode_arg,
            line_following_node_launch,
            large_models_launch,
            llm_visual_patrol_node,
            ]
```

This function is used to configure and launch actions in a Launch file.

`mode = LaunchConfiguration('mode', default=1)`: Defines a Launch parameter named mode with a default value of 1.

`mode_arg = DeclareLaunchArgument('mode', default_value=mode)`: Declares the mode parameter and adds it to the Launch description.

`large_models_package_path`: Retrieves the shared directory path of the `large_models` package.

`line_following_node_launch`: Uses `IncludeLaunchDescription` to include the `line_following_node.launch.py` file.

`large_models_launch`: Uses `IncludeLaunchDescription` to include the `start.launch.py` launch file from the `large_models` package and passes the mode parameter.

`llm_vision_pratrol`: Defines a ROS2 node from the `large_models` package that runs the `llm_vision_pratrol` executable and prints its output to the screen.

Finally, returns a list containing all Launch actions.

* **Python File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/llm_visual_patrol.py**.

1\. Define Prompt Template (PROMPT)

```python
else:
    PROMPT = '''
**Role
You are a smart robot that generates corresponding JSON commands based on user input.

**Requirements
- For every user input, search for matching commands in the action function library and output the corresponding JSON instruction.
- For each action sequence, craft a witty and creative response (10 to 30 characters) to make interactions delightful.
- Directly return the JSON result — do not include any explanations or extra text.
- There are four target colors: red, green, blue, and black.
- Format:
{
  "action": ["xx", "xx"],
  "response": "xx"
}

**Special Notes
The "action" field should contain a list of function names as strings, ordered by execution. If no matching action is found, output an empty list: [].
The "response" field should provide a concise and charming reply, staying within the word and tone guidelines.

**Action Function Library
Follow a line of a given color: line_following('black')

**Example
Input: Follow the red line
Output:
{
  "action": ["line_following('red')"],
  "response": "Roger that!"
}
    '''
```

2\. Class Initialization

```python
def __init__(self, name):
    rclpy.init()
    super().__init__(name)

    self.action = []
    self.stop = True
    self.llm_result = ''
    # self.llm_result = '{"action": ["line_following(\'black\')"], "response": "ok！"}'
    self.running = True
    self.action_finish = False
    self.play_audio_finish = False

    timer_cb_group = ReentrantCallbackGroup()
    self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
    self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
    self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=timer_cb_group)
    self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
    self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
    self.awake_client.wait_for_service()
    self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
    self.set_mode_client.wait_for_service()

    self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
    self.set_model_client.wait_for_service()
    self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
    self.set_prompt_client.wait_for_service()
    self.enter_client = self.create_client(Trigger, 'line_following/enter')
    self.enter_client.wait_for_service()
    self.start_client = self.create_client(SetBool, 'line_following/set_running')
    self.start_client.wait_for_service()
    self.set_target_client = self.create_client(SetColor, 'line_following/set_color')
    self.set_target_client.wait_for_service()

    self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

Set state variables, including stop flags, LLM processing outputs, and runtime status. Create publishers for TTS text output and subscribers for agent responses, wake-word signals, and audio-playback completion. Service-client requests cover entering tracking mode, updating the running state, and specifying the tracking color, providing the communication and control interface for the system.

3\. Set LLM Model

```python
def init_process(self):
    self.timer.cancel()

    msg = SetModel.Request()
    msg.model = llm_model
    msg.model_type = 'llm'
    msg.api_key = api_key 
    msg.base_url = base_url
    self.send_request(self.set_model_client, msg)

    msg = SetString.Request()
    msg.data = PROMPT
    self.send_request(self.set_prompt_client, msg)

    init_finish = self.create_client(Trigger, 'line_following/init_finish')
    init_finish.wait_for_service()
    self.send_request(self.enter_client, Trigger.Request())
    speech.play_audio(start_audio_path)
    threading.Thread(target=self.process, daemon=True).start()
    self.create_service(Empty, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)
```

4\. `play_audio_finish_callback` Method

```python
def play_audio_finish_callback(self, msg):
    self.play_audio_finish = msg.data
```

The method handles the callback after audio playback and re-enables the voice wake-up functionality.

5\. `process` Method

```python
def process(self):
    while self.running:
        if self.llm_result:
            msg = String()
            if 'action' in self.llm_result: # If there is a corresponding action returned, extract and process it
                result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                if 'action' in result:
                    text = result['action']
                    # Use regular expressions to extract all strings inside parentheses
                    pattern = r"line_following\('([^']+)'\)"
                    # Use re.search to find the matching result
                    for i in text:
                        match = re.search(pattern, i)
                        # Extract the result
                        if match:
                            # Get all argument parts, precisely content inside parentheses
                            color = match.group(1)
                            self.get_logger().info(str(color))
                            color_msg = SetColor.Request()
                            color_msg.data = color
                            self.send_request(self.set_target_client, color_msg)
                            # Start sorting
                            start_msg = SetBool.Request()
                            start_msg.data = True 
                            self.send_request(self.start_client, start_msg)
                if 'response' in result:
                    msg.data = result['response']
            else: # If there is no corresponding action, just respond
                msg.data = self.llm_result
            self.tts_text_pub.publish(msg)
            self.action_finish = True
            self.llm_result = ''
        else:
            time.sleep(0.01)
        if self.play_audio_finish and self.action_finish:
            self.play_audio_finish = False
            self.action_finish = False
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)
            # msg = SetInt32.Request()
            # msg.data = 2
            # self.send_request(self.set_mode_client, msg)
            self.stop = False
    rclpy.shutdown()
```

By continuously monitoring the large model output, the system checks for any instruction that contains the `line_following` action. Once detected, it uses a regular expression to extract the color parameter and sends it to the line-following module as the target color, then activates the tracking function. If a text response is generated, the system delivers it through TTS for voice feedback. During execution, it coordinates action handling with the completion status of audio playback. After the task is finished, it resets the status flags and re-enables the voice-wake function, ready for the next command.

### 19.2.5 Color Tracking

#### 19.2.5.1 Program Overview

When the program starts, the voice device will announce **I'm ready**. To activate the voice device, say the designated wake word **Hello Hiwonder**. Upon successful activation, the voice device will respond with **I’m here**. Once activated, voice commands can be issued, for example, “Follow the red object.” The terminal displays the recognized command, and the voice device provides a processed response. The robot then autonomously detects the red object using its camera and starts tracking it.

#### 19.2.5.2 Preparation

* **Version Confirmation**

Before starting this feature, ensure that the microphone version configuration in the system is correct. For more details, refer to section [19.2.2 Version Confirmation](#anther19.2.2).

* **Configure Large Model API-KEY**

Refer to the section [19.2.1 Large Model API Key Setup](#anther19.2.1) to set up the large model key.

#### 19.2.5.3 Operation Steps

> [!NOTE]
>
> * **Command input is case-sensitive and space-sensitive.**
>
> * **The robot must be connected to the Internet, either in STA (LAN) mode or AP (direct connection) mode via Ethernet.**

1. Click the terminal <img  src="../_static/media/chapter_19/section_3/media/image25.png"  /> on the left side of the system interface to open the command line. Enter the command and press **Enter** to disable the app auto-start service.

```bash
sudo systemctl stop start_app_node.service
```

2. Open the command line terminal <img src="../_static/media/chapter_19/section_3/media/image14.png"   /> from the left side of the system interface. In the terminal window, enter the following command and press **Enter** to launch the color tracking feature.

```bash
ros2 launch large_models_examples llm_color_track.launch.py
```

3) When the command line displays the output shown below and announces **I’m ready**, the voice device has completed initialization. At this point, say the wake word **Hello Hiwonder**.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image36.png" style="width:700px" />

4) After the program has loaded successfully, the camera feed will appear on screen.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image37.png" style="width:700px" />

5) When the command line shows the output below and the voice device announces **I’m here**, the voice device has been successfully activated. The system will begin recording voice commands.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image38.png" style="width:700px" />

6) When the terminal displays the output shown below, it indicates that the voice device has printed the recognized speech. Now, say the command “Follow the red object.”

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image39.png" style="width:700px" />

7) Upon successful recognition by the speech recognition service of a cloud-based large speech model, the parsed command will be displayed under the **publish_asr_result** output in the terminal.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image39.png" style="width:700px" />

8. When the command line displays the output shown below, the cloud-based large language model has been successfully invoked. It processes the user’s command, provides a verbal response, and generates actions corresponding to the semantic meaning of the command.

   The response is automatically generated by the large model. Only the semantic accuracy of the reply is guaranteed, while the wording and formatting may vary.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image40.png" style="width:700px" />

9) Then, the robot will detect the red object in its camera feed and begin tracking it in real time.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image41.png" style="width:700px" />

10) To stop the feature, refer to Step 4 to reactivate the voice device again with the wake word to begin a new interaction cycle.

<img class="common_img" src="../_static/media/chapter_19/section_3/media/image42.png" style="width:700px" />

11) To exit the feature, press **Ctrl+C** in the terminal. If the feature does not shut down immediately, press **Ctrl+C** multiple times. If exiting fails, open a new terminal and enter the command to terminate all currently running ROS nodes and related processes.

```bash
~/.stop_ros.sh
```

#### 19.2.5.4 Program Outcome

Once the feature is activated, commands can be freely issued, for example, **Follow the red object**. The robot uses its camera feed to detect and track the red object. Similarly, commands such as “Follow the blue object” or “Follow the green object” can be used to have the robot detect and track objects of the corresponding colors.

#### 19.2.5.5 Program Analysis

* **Launch File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/llm_color_track.launch.py**.

Define `launch_setup` Function

```python
def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    app_package_path = get_package_share_directory('app')
    large_models_package_path = get_package_share_directory('large_models')

    object_tracking_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/object_tracking_node.launch.py')),
        launch_arguments={
            'debug': 'true',
        }.items(),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    llm_color_track_node = Node(
        package='large_models_examples',
        executable='llm_color_track',
        output='screen',
    )

    return [mode_arg,
            object_tracking_node_launch,
            large_models_launch,
            llm_color_track_node,
            ]
```

This function is used to configure and launch actions in a Launch file.

`mode = LaunchConfiguration('mode', default=1)`: Defines a Launch parameter named mode with a default value of 1.

`mode_arg = DeclareLaunchArgument('mode', default_value=mode)`: Declares the mode parameter and adds it to the Launch description.

`objet_sorting_launch`: Uses `IncludeLaunchDescription` to include the `object_tracking_node.launch.py` file from the `large_models` package and passes the mode parameter.

`large_models_launch`: Uses `IncludeLaunchDescription` to include the `start.launch.py` file from the `large_models` package and passes the mode parameter.

`llm_color_track_node`: Defines a ROS2 node from the `large_models` package that runs the `llm_color_track` executable and prints its output to the screen.

Finally, returns a list containing all Launch actions.

* **Python File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/llm_color_track.py**

1\. Define Prompt Template (PROMPT)

```python
else:
    PROMPT = '''
**Role
You are an intelligent companion robot. Your job is to generate corresponding JSON commands based on the user’s input.

**Requirements
- For every user input, search the action function library for matching commands and return the corresponding JSON instruction.
- Craft a witty, ever-changing, and concise response (between 10 to 30 characters) for each action sequence to make interactions lively and fun.
- Output only the JSON result — do not include explanations or any extra text.
- Output format:{"action": ["xx", "xx"], "response": "xx"}

**Special Notes
The "action" key holds an array of function name strings arranged in execution order. If no match is found, return an empty array [].
The "response" key must contain a cleverly worded, short reply (10–30 characters), adhering to the tone and length guidelines above.

**Action Function Library
Track an object of a specific color: color_track('red')

**Example
Input: Track a green object
Output:
{"action": ["color_track('green')"], "response": "Got it!"}
    '''
```

2\. Class Initialization

```python
def __init__(self, name):
    rclpy.init()
    super().__init__(name)

    self.action = []
    self.stop = True
    self.llm_result = ''
    # self.llm_result = '{"action": ["color_track(\'blue\')"], "response": "ok！"}'
    self.running = True
    self.action_finish = False
    self.play_audio_finish = False

    timer_cb_group = ReentrantCallbackGroup()
    self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
    self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
    self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1, callback_group=timer_cb_group)
    self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
    self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
    self.awake_client.wait_for_service()
    self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
    self.set_mode_client.wait_for_service()

    self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
    self.set_model_client.wait_for_service()
    self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
    self.set_prompt_client.wait_for_service()

    self.enter_client = self.create_client(Trigger, 'object_tracking/enter')
    self.enter_client.wait_for_service()
    self.start_client = self.create_client(SetBool, 'object_tracking/set_running')
    self.start_client.wait_for_service()
    self.set_target_client = self.create_client(SetColor, 'object_tracking/set_color')
    self.set_target_client.wait_for_service()

    self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

By initializing state parameters, including the action list, stop flag, LLM processing results, and completion status, a TTS text publisher and subscribers are created to receive agent processing results, voice wake signals, and audio playback completion status. Service client requests are also established for voice wake, node configuration, model setup, prompt settings, and object tracking functions, including entering tracking mode, setting run status, and specifying the target color. Finally, a timer is created to start the initialization process.

3\. Set LLM Model

```python
def init_process(self):
    self.timer.cancel()
    msg = SetModel.Request()
    msg.model = llm_model
    msg.model_type = 'llm'
    msg.api_key = api_key 
    msg.base_url = base_url
    self.send_request(self.set_model_client, msg)

    msg = SetString.Request()
    msg.data = PROMPT
    self.send_request(self.set_prompt_client, msg)

    init_finish = self.create_client(Trigger, 'object_tracking/init_finish')
    init_finish.wait_for_service()
    self.send_request(self.enter_client, Trigger.Request())
    speech.play_audio(start_audio_path)
    threading.Thread(target=self.process, daemon=True).start()
    self.create_service(Empty, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)
```

4\. `play_audio_finish_callback` Method

```python
def play_audio_finish_callback(self, msg):
    self.play_audio_finish = msg.data
```

The method handles the callback after audio playback and re-enables the voice wake-up functionality.

5\. `process` Method

```python
def process(self):
    while self.running:
        if self.llm_result:
            msg = String()
            if 'action' in self.llm_result: # If there is a corresponding action returned, extract and process it
                result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                if 'action' in result:
                    text = result['action']
                    # Use regular expressions to extract all strings inside parentheses
                    pattern = r"color_track\('([^']+)'\)"
                    # Use re.search to find the matching result
                    for i in text:
                        match = re.search(pattern, i)
                        # Extract the result
                        if match:
                            # Get all argument parts, precisely content inside parentheses
                            color = match.group(1)
                            self.get_logger().info(str(color))
                            color_msg = SetColor.Request()
                            color_msg.data = color
                            self.send_request(self.set_target_client, color_msg)
                            # Start sorting
                            start_msg = SetBool.Request()
                            start_msg.data = True 
                            self.send_request(self.start_client, start_msg)
                if 'response' in result:
                    msg.data = result['response']
            else: # If there is no corresponding action, just respond
                msg.data = self.llm_result
            self.tts_text_pub.publish(msg)
            self.action_finish = True
            self.llm_result = ''
        else:
            time.sleep(0.01)
        if self.play_audio_finish and self.action_finish:
            self.play_audio_finish = False
            self.action_finish = False
            # msg = SetInt32.Request()
            # msg.data = 1
            # self.send_request(self.set_mode_client, msg)
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)
            self.stop = False
    rclpy.shutdown()
```

By continuously monitoring the large model output, the system checks for any instruction that contains the `color_track` action. Once detected, it uses a regular expression to extract the color parameter and sends it to the color-tracking module as the target color, then activates the tracking function. If a text response is generated, the system delivers it through TTS for voice feedback. During execution, it coordinates action handling with the completion status of audio playback. After the task is finished, it resets the status flags and re-enables the voice-wake function, ready for the next command.



## 19.3 Embodied AI Applications

<p id ="anther19.3.1"></p>

### 19.3.1 Large Model API Key Setup

> [!NOTE]
>
> **This section requires registering on the official OpenAI website and obtaining an API key for accessing large language models.**

#### 19.3.1.1 OpenAI Account Registration and Deployment

1) Copy and open the following URL: https://platform.openai.com/docs/overview, then click the **Sign Up** button in the upper-right corner.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image1.png"  />

2) Register and log in using a Google, Microsoft, or Apple account, as prompted.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image2.png" style="width:600px" />

3) After logging in, click the **Settings** button, then go to **Billing**, and click **Payment Methods** to add a payment method. The payment is used to purchase **tokens**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image3.png"  />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image4.png" style="width:700px" />

4) After completing the preparation steps, click **API Keys** and create a new key. Follow the prompts to fill in the required information, then save the key for later use.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image5.png"  />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image6.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image7.png" style="width:500px" />

5) The creation and deployment of the large model have been completed, and this API will be used in the following sections.

#### 19.3.1.2 OpenRouter Account Registration and Deployment

1) Copy the URL https://openrouter.ai/ into a browser and open the webpage. Click **Sign in** and register or sign in using a Google account or another available account.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image8.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image9.png"  />

2) After logging in, click the icon in the top-right corner, then select **Credits** to add a payment method.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image10.png" style="width:500px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image11.png" style="width:700px" />

3) Create an API key. Go to **API Keys**, then click **Create Key**. Follow the prompts to generate a key. Save the API key securely for later use.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image12.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image13.png" style="width:700px" />

The creation and deployment of the large model have been completed, and this API will be used in the following sections.

#### 19.3.1.3 API Configuration

1. Click <img src="../_static/media/chapter_19/section_4/media/image14.png"   /> to open a terminal and enter the following command to open the configuration file. Press the **i** key to enter input mode.

```bash
vim /home/ubuntu/ros2_ws/src/large_models/large_models/large_models/config.py
```

2. Fill in the obtained Large Model API Key in the corresponding parameter, as shown in the red box in the figure below.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image15.png"  />

3. Press **Esc**, then enter the command and press **Enter** to save and exit the configuration file.

```bash
:wq
```

<p id ="anther19.3.2"></p>

### 19.3.2 Version Confirmation

Before starting features, verify that the correct microphone configuration is set in the system.

1. After remotely logging in via NoMachine, click the desktop icon <img src="../_static/media/chapter_19/section_4/media/image16.png"  /> to access the configuration interface.

2. Select the appropriate microphone version configuration according to the hardware.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image17.png"  />

3. If using the AI Voice Interaction Box, select **WonderEcho Pro** as the microphone type, as shown in the figure below.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image18.png"  style="width:300px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image19.png"  />

4. For the Six-Microphone Array, select **xf** as the microphone type as shown in the figure.

   <img class="common_img" src="../_static/media/chapter_19/section_4/media/image20.png"  style="width:300px" />

   <img class="common_img" src="../_static/media/chapter_19/section_4/media/image21.png"  />

5. Click **Save**.

   <img class="common_img" src="../_static/media/chapter_19/section_4/media/image22.png"  />

6. After the **Save Success** notification appears, click **Apply**.

   <img class="common_img" src="../_static/media/chapter_19/section_4/media/image23.png"  />

7. Finally, click **Quit** to exit the software interface.

   <img class="common_img" src="../_static/media/chapter_19/section_4/media/image24.png"  />


### 19.3.3 Real-Time Detection

#### 19.3.3.1 Program Overview

When the program starts, the voice device will announce **I'm ready**. To activate the voice device, speak the designated wake words **Hello Hiwonder**. Upon successful activation, the voice device will respond with **I’m here**. Then, voice commands can be used to control the robot, for example, **Tell me what you saw**. The terminal will display the recognized speech, the voice device will respond with a generated reply after processing, and the robot will autonomously analyze the camera feed and describe the content of the scene.

#### 19.3.3.2 Preparation

* **Version Confirmation**

Before starting this feature, ensure that the microphone version configuration in the system is correct. For more details, refer to section [19.3.2 Version Confirmation](#anther19.3.2).

* **Configure Large Model API-KEY**

Refer to the section [19.3.1 Large Model API Key Setup](#anther19.3.1) to set up the large model key.

#### 19.3.3.3 Operation Steps

1. Click the terminal <img src="../_static/media/chapter_19/section_4/media/image25.png"  /> on the left side of the system interface to open the command line. Enter the command and press **Enter** to disable the app auto-start service.

```bash
sudo systemctl stop start_app_node.service
```

2. Open the command line terminal <img src="../_static/media/chapter_19/section_4/media/image14.png"   /> from the left side of the system interface. In the terminal window, enter the following command and press **Enter** to launch the real-time detection feature.

```bash
ros2 launch large_models_examples vllm_with_camera.launch.py
```

3) When the command line displays the output shown below and announces **I’m ready**, the voice device has completed initialization. At this point, say the wake word **Hello Hiwonder**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image26.png" style="width:700px" />

4) When the command line shows the output below and the voice device announces **I’m here**, it has been successfully activated. The system will begin recording voice commands.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image27.png" style="width:700px" />

5) When the command line displays the output shown below, it indicates that the voice device has printed the recognized speech. At this point, user command recording begins. Next, say the command **Tell me what you saw**, and wait for the large model to process it. When the command line shows the output below, it indicates that the cloud-based speech large model service has successfully processed the audio command, with the recognition result available in **publish_asr_result**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image28.png" style="width:700px" />

6) Upon receiving user input shown in the figure, the terminal will display output indicating that the cloud-based large language model has been successfully invoked. The model will interpret the command and generate a language response.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image29.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image30.png" style="width:700px" />

The response is automatically generated by the large model. Only the semantic accuracy of the reply is guaranteed, while the wording and formatting may vary.

7) When the command line displays the output shown below, the current dialogue session has ended. Refer to Step 4 to reactivate the voice device with the wake word and start a new dialogue session.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image31.png" style="width:700px" />

8) To exit the feature, press **Ctrl+C** in the terminal. If the feature does not exit immediately, press **Ctrl+C** multiple times.

#### 19.3.3.4 Program Outcome

Once the feature is activated, any command can be issued to the robot, for example: “Tell me what you saw.” The robot will automatically analyze the scene within its camera view, process the information, and describe the current environment in detail.

#### 19.3.3.5 Program Analysis

* **Launch File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/vllm_with_camera.launch.py**.

Define `launch_setup` Function

```python
def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)

    camera_topic = LaunchConfiguration('camera_topic', default='depth_cam/rgb0/image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    controller_package_path = get_package_share_directory('controller')
    peripherals_package_path = get_package_share_directory('peripherals')
    large_models_package_path = get_package_share_directory('large_models')

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
    )
    
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode, 'camera_topic': camera_topic}.items(),
    )

    vllm_with_camera_node = Node(
        package='large_models_examples',
        executable='vllm_with_camera',
        output='screen',
        parameters=[{"camera_topic": camera_topic}],
    )

    return [mode_arg,
            camera_topic_arg,
            controller_launch,
            depth_camera_launch,
            large_models_launch,
            vllm_with_camera_node,
            ]
```

This function is used to configure and launch actions in a Launch file.

`mode = LaunchConfiguration('mode', default=1)`: Defines a Launch parameter named mode with a default value of 1.

`mode_arg = DeclareLaunchArgument('mode', default_value=mode)`: Declares the mode parameter and adds it to the Launch description.

`depth_camera_launch`: Uses `IncludeLaunchDescription` to include the `depth_camera.launch.py Launch` file from the `large_models` package and passes the mode parameter.

`controller_launch`: Uses `IncludeLaunchDescription` to include the `controller.launch.py` launch file from the `large_models` package and passes the mode parameter.

`large_models_launch`: Uses `IncludeLaunchDescription` to include the `start.launch.py` file from the `large_models` package and passes the mode parameter.

`vllm_with_camera`: Defines a ROS2 node from the large_models package that runs the vllm_with_camera executable and prints its output to the screen.

Finally, returns a list containing all Launch actions.

* **Python File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/vllm_with_camera.py**.

1\. Class Initialization

```python
def __init__(self, name):
    rclpy.init()
    super().__init__(name)

    # Initialize variables
    self.image_queue = queue.Queue(maxsize=2)
    self.set_above = False
    self.vllm_result = ''
    self.running = True
    self.action_finish = False
    self.play_audio_finish = False
    self.bridge = CvBridge()
    self.client = speech.OpenAIAPI(api_key, base_url)

    # Add debug flags
    self.image_received = False
    self.first_image_time = None

    # Declare parameters
    self.declare_parameter('camera_topic', '/ascamera/camera_publisher/rgb0/image')
    camera_topic = self.get_parameter('camera_topic').value

    # Print camera topic information
    self.get_logger().info(f'Camera topic: {camera_topic}')

    # Create callback group
    timer_cb_group = ReentrantCallbackGroup()

    # Create publishers
    self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
    self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)

    # Create subscribers
    self.create_subscription(Image, camera_topic, self.image_callback, 1)
    self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
    self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)

    # Create clients
    self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
    self.awake_client.wait_for_service()
    self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
    self.set_model_client.wait_for_service()
    self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
    self.set_mode_client.wait_for_service()
    self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
    self.set_prompt_client.wait_for_service()

    # Create initialization timer
    self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

Initialize state parameters, including the action list, VLLM processing results, running status, and interrupt flags. Create publishers, such as TTS text and chassis motion control, and subscribers to receive agent processing results, voice wake signals, and audio playback completion status. Set up service client requests for model configuration, voice wake enablement, and prompt setup, then create a timer to start the initialization process.

2\. `get_node_state` Method

```python
def get_node_state(self, request, response):
    return response
```

Service callback function that returns an empty response, used for querying the node’s status.

3\. `init_process` Method

```python
def init_process(self):
    """Initialization process"""
    self.timer.cancel()

    # Set the model
    msg = SetModel.Request()
    msg.model_type = 'vllm'
    if os.environ['ASR_LANGUAGE'] == 'Chinese':
        msg.model = stepfun_vllm_model
        msg.api_key = stepfun_api_key
        msg.base_url = stepfun_base_url
    else:
        msg.model = vllm_model
        msg.api_key = vllm_api_key
        msg.base_url = vllm_base_url
    self.send_request(self.set_model_client, msg)

    # Set the prompt
    msg = SetString.Request()
    msg.data = VLLM_PROMPT
    self.send_request(self.set_prompt_client, msg)

    set_servo_position(self.joints_pub, 1.0,
                       ((1, 500), (2, 645), (3, 135), (4, 80), (5, 500), (10, 220)))

    # Play startup audio
    speech.play_audio(start_audio_path)

    # Wait for the first image before starting the processing thread
    self.get_logger().info('Waiting for first image...')
    threading.Thread(target=self.wait_and_start_process, daemon=True).start()

    # Create service
    self.create_service(Empty, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
```

Intelligently select the language model configuration, set up a multilingual visual prompt template, initialize and play the start audio cue, launch the processing thread, create a service indicating initialization completion, and finally output the startup log.

4\. `play_audio_finish_callback` Method

```python
def play_audio_finish_callback(self, msg):
    self.play_audio_finish = msg.data
```

It sends a request to enable the wake-up feature once audio playback is complete.

5\. `process` Method

```python
def process(self):
    """Main processing loop - display images and handle VLLM results"""
    # Create OpenCV window
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', display_size[0], display_size[1])

    while self.running:
        try:
            # Get image from queue with timeout to avoid blocking indefinitely
            image = self.image_queue.get(timeout=0.1)

            # Process VLLM results
            if self.vllm_result:
                msg = String()
                msg.data = self.vllm_result
                self.tts_text_pub.publish(msg)
                self.vllm_result = ''
                self.action_finish = True

            # Handle audio playback completion
            if self.play_audio_finish and self.action_finish:
                self.play_audio_finish = False
                self.action_finish = False
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)

            # Convert color space and resize
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            display_image = cv2.resize(bgr_image, (display_size[0], display_size[1]))

            # Display image
            cv2.imshow('image', display_image)

            # Set window position (execute only once)
            if not self.set_above:
                cv2.moveWindow('image', 1920 - display_size[0], 0)
                try:
                    os.system("wmctrl -r image -b add,above")
                except Exception as e:
                    self.get_logger().warn(f'Failed to set window always on top: {e}')
                self.set_above = True

        except queue.Empty:
            # Continue loop if queue is empty
            pass
        except Exception as e:
            self.get_logger().error(f'Error in process loop: {e}')

        # Check keyboard input
        k = cv2.waitKey(1)
        if k == 27 or k == ord('q'):  # ESC or Q key to quit
            self.get_logger().info('User requested quit')
            break

    # Cleanup
    cv2.destroyAllWindows()
    self.running = False
```

Continuously retrieve image frames from the image queue and monitor the outputs of the vision-language model. When a valid result is detected, convert it into a speech message and publish it via TTS, coordinating the completion status of visual recognition and voice feedback. After the task is completed, re-enable the voice wake-up function.

6\. `image_callback` Method

```python
def image_callback(self, ros_image):
    """Image callback function"""
    try:
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)

        # Record the first received image
        if not self.image_received:
            self.image_received = True
            self.first_image_time = time.time()
            self.get_logger().info(f'First image received! Size: {rgb_image.shape}')

        # If the queue is full, discard the oldest image
        if self.image_queue.full():
            self.image_queue.get()

        # Put the image into the queue
        self.image_queue.put(rgb_image)

    except Exception as e:
        self.get_logger().error(f'Error in image callback: {e}')
```

It converts the received ROS image messages to `NumPy` arrays and stores them in the queue.

### 19.3.4 Vision Tracking

The large model used in this lesson operates online, requiring a stable network connection for the controller during the operation.

#### 19.3.4.1 Program Overview

When the program starts, the voice device will announce **I'm ready**. To activate the voice device, speak the designated wake words **Hello Hiwonder**. Upon successful activation, the voice device will respond with **I’m here**. Once activated, voice commands can be issued, for example, **Follow the person in white in front**. The terminal will display the recognized command, and the voice device will respond with a generated reply after processing and execute corresponding actions.

#### 19.3.4.2 Preparation

* **Version Confirmation**

Before starting this feature, ensure that the microphone version configuration in the system is correct. For more details, refer to section [19.3.2 Version Confirmation](#anther19.3.2).

* **Configure Large Model API-KEY**

Refer to the section [19.3.1 Large Model API Key Setup](#anther19.3.1) to set up the large model key.

#### 19.3.4.3 Operation Steps

> [!NOTE]
>
> **Command input is case-sensitive and space-sensitive.**
>
> **The robot must be connected to the Internet, either in STA (LAN) mode or AP (direct connection) mode via Ethernet.**

1. Click the terminal <img src="../_static/media/chapter_19/section_4/media/image25.png"  /> on the left side of the system interface to open the command line. Enter the command and press **Enter** to disable the app auto-start service.

```bash
sudo systemctl stop start_app_node.service
```

2) Open the command line terminal <img src="../_static/media/chapter_19/section_4/media/image14.png"   /> from the left side of the system interface. In the terminal window, enter the following command and press **Enter** to launch the vision tracking feature.

```bash
ros2 launch large_models_examples vllm_track.launch.py
```

3) When the command line displays the output shown below and the device announces **I'm ready**, it indicates that the voice device has completed initialization and the YOLO model has also been initialized. At this point, say the wake word **Hello Hiwonder** to activate the voice device.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image32.png" style="width:700px" />

4) When the command line shows the output below and the voice device announces **I’m here**, it has been successfully activated. The system will begin recording voice commands.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image33.png" style="width:700px" />

5) When the command line displays the output shown below, it indicates that the voice device has printed the recognized speech. At this point, user command recording begins. Next, say the command **Follow the person in white in front**, and wait for the large model to process it. When the command line shows the output below, it indicates that the cloud-based speech large model service has successfully processed the audio command, with the recognition result available in **publish_asr_result**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image34.png"  style="width:700px" />

6. When the command line displays the output shown below, the cloud-based large language model has been successfully invoked. It processes the user’s command, provides a verbal response, and generates actions corresponding to the semantic meaning of the command.

   The response is automatically generated by the large model. Only the semantic accuracy of the reply is guaranteed, while the wording and formatting may vary.

7. When the terminal shows the output shown in the figure, indicating the end of one interaction cycle, the system is ready for the next round. To initiate another interaction, repeat step 4 by speaking the wake words again.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image35.png" style="width:700px" />

8) To exit the feature, press **Ctrl+C** in the terminal. If the feature does not exit immediately, press **Ctrl+C** multiple times.

#### 19.3.4.4 Program Outcome

Once the feature is activated, any command can be issued to the robot, such as **Follow the person in white in front**. The robot will detect the person wearing white in its camera view and stop after reaching a preset distance.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image36.png" style="width:700px" />

#### 19.3.4.5 Program Analysis

* **Launch File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/vllm_track.launch.py**.

Define `launch_setup` Function

```python
def launch_setup(context):
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    
    slam_package_path = get_package_share_directory('slam')
    large_models_package_path = get_package_share_directory('large_models') 
    
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': 'false',
            'master_name': os.environ['MASTER'],
            'robot_name': os.environ['HOST']
        }.items(),
    )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    vllm_track_node = Node(
        package='large_models_examples',
        executable='vllm_track',
        output='screen',
    )

    # rqt
    calibrate_rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='calibrate_rqt_reconfigure'
    )

    return [mode_arg,
            base_launch,
            large_models_launch,
            vllm_track_node,
            # calibrate_rqt_reconfigure_node,
            ]
```

This function is used to configure and launch actions in a Launch file.

`mode = LaunchConfiguration('mode', default=1)`: Defines a Launch parameter named mode with a default value of 1.

`mode_arg = DeclareLaunchArgument('mode', default_value=mode)`: Declares the mode parameter and adds it to the Launch description.

`slam_package_path和large_models_package_path`: Retrieves the shared directory path of the `slam` package and the `large_models` package.

`base_launch`: Includes the `robot.launch.py` file from the `slam` package and passes the parameters `sim`, `master_name`, and `robot_name`.

`large_models_launch`: Includes the `start.launch.py` launch file from the `large_models` package and passes the node parameter.

`llm_track_node`: Defines a ROS2 node from the `large_models` package that runs the `vllm_track` executable and prints its output to the screen.

`calibrate_rqt_reconfigure_node`: An `rqt_reconfigure` node used for debugging.

Finally, returns a list containing all Launch actions.

* **Python File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/vllm_track.py**.

1\. Define Prompt Template (PROMPT)

```python
else:
    PROMPT = '''
**Role
You are a smart car with advanced visual recognition capabilities. Your task is to analyze an image sent by the user, perform object detection, and follow the detected object. Finally, return the result strictly following the specified output format.

Step 1: Understand User Instructions
You will receive a sentence. From this sentence, extract the object name to be detected.
Note: Use English for the object value, do not include any objects not explicitly mentioned in the instruction.

Step 2: Understand the Image
You will also receive an image. Locate the target object in the image and return its coordinates as the top-left and bottom-right pixel positions in the form [xmin, ymin, xmax, ymax].
Note: If the object is not found, then "xyxy" should be an empty list: [], only detect and report objects mentioned in the user instruction.The coordinates (xmin, ymin, xmax, ymax) must be normalized to the range [0, 1]

**Important: Accurately understand the spatial position of the object. The "response" must reflect both the user's instruction and the detection result.

**Output Format (strictly follow this format, do not output anything else.The coordinates (xmin, ymin, xmax, ymax) must be normalized to the range [0, 1])
{
    "object": "name", 
    "xyxy": [xmin, ymin, xmax, ymax],
    "response": "reflect both the user's instruction and the detection result (5-30 characters)"
}

**Example
Input: track the person
Output:
{
    "object": "person",
    "xyxy": [0.1, 0.3, 0.4, 0.6],
    "response": "I have detected a person in a white T-shirt and will track him now."
}
    '''
```

2\. Class Initialization

```python
def __init__(self, name):
    rclpy.init()
    super().__init__(name)
    self.machine_type = os.environ['MACHINE_TYPE']
    self.fps = fps.FPS() # Frame rate counter
    self.image_queue = queue.Queue(maxsize=2)
    self.vllm_result = ''
    # self.vllm_result = '''json{"object":"Red block", "xyxy":[521, 508, 637, 683]}'''
    self.running = True
    self.data = []
    self.start_track = False
    self.bridge = CvBridge()
    #cv2.namedWindow('image', 0)
    #cv2.moveWindow('image', 1920 - 640, 0)
    #cv2.waitKey(10)
    #os.system("wmctrl -r image -b add,above")
    self.camear_type = os.environ['DEPTH_CAMERA_TYPE']
    self.track = ObjectTracker(use_mouse=False, automatic=True, log=self.get_logger())
    timer_cb_group = ReentrantCallbackGroup()
    self.client = speech.OpenAIAPI(api_key, base_url)
    self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # Chassis control
    self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
    self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
    self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
    self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)

    self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
    self.awake_client.wait_for_service()
    self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
    self.set_model_client.wait_for_service()
    self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
    self.set_prompt_client.wait_for_service()

    image_sub = message_filters.Subscriber(self, Image, 'ascamera/camera_publisher/rgb0/image')
    depth_sub = message_filters.Subscriber(self, Image, 'ascamera/camera_publisher/depth0/image_raw')

    self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

    # Synchronize timestamps with a time tolerance of 0.03s
    sync = message_filters.ApproximateTimeSynchronizer([depth_sub, image_sub], 3, 0.02)
    sync.registerCallback(self.multi_callback)

    # Define PID parameters
    # 0.07, 0, 0.001
    self.pid_params = {
        'kp1': 0.01, 'ki1': 0.0, 'kd1': 0.00,
        'kp2': 0.002, 'ki2': 0.0, 'kd2': 0.0,
    }

    # Dynamically declare parameters
    for param_name, default_value in self.pid_params.items():
        self.declare_parameter(param_name, default_value)
        self.pid_params[param_name] = self.get_parameter(param_name).value

    self.track.update_pid([self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
                  [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']])

    # Callback function for dynamic updates
    self.add_on_set_parameters_callback(self.on_parameter_update)

    self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

By initializing the robot's chassis type, frame rate monitor, and image queue, publishers for TTS text output and chassis motion control are created, along with subscribers for voice wake-up detection and receiving agent processing results. Service client requests are set up for enabling voice wake-up and configuring models. Message filters synchronize depth and RGB images. PID parameters are configured for precise target tracking control with support for dynamic parameter adjustment, and finally, a timer is created to start the initialization process.

3\. `init_process` Method

```python
def init_process(self):
    self.timer.cancel()

    self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
    set_servo_position(self.joints_pub, 1, ((10, 200), (5, 500), (4, 220), (3, 100), (2, 665), (1, 500)))

    msg = SetModel.Request()
    msg.model_type = 'vllm'
    if os.environ['ASR_LANGUAGE'] == 'Chinese':
        msg.model = stepfun_vllm_model
        msg.api_key = stepfun_api_key
        msg.base_url = stepfun_base_url
    else:
        msg.model = vllm_model
        msg.api_key = vllm_api_key
        msg.base_url = vllm_base_url
    self.send_request(self.set_model_client, msg)

    msg = SetString.Request()
    msg.data = PROMPT
    self.send_request(self.set_prompt_client, msg)

    self.mecanum_pub.publish(Twist())
    time.sleep(1.8)
    speech.play_audio(start_audio_path)
    threading.Thread(target=self.process, daemon=True).start()
    self.create_service(Empty, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    self.get_logger().info('\033[1;32m%s\033[0m' % PROMPT)
```

By removing the timer, a publisher is created. Based on the system's language setting, the appropriate visual language model configuration is selected, and a dedicated prompt template is set. The stop motion command for the chassis is published, and the start prompt audio is played. At the same time, the processing thread is started, the initialization completion status service is created, and finally, the startup log information is output.

4\. `process` Method

```python
def process(self):
    box = ''

    while self.running:

        image, depth_image = self.image_queue.get(block=True)

        if self.vllm_result:
            try:
                self.vllm_result = json.loads(self.vllm_result)
                box = self.vllm_result['xyxy']
                if box:
                    if self.camear_type == 'aurora':
                        w_ = 640
                        h_ = 400
                    else:
                        w_ = 640
                        h_ = 480
                    if os.environ["ASR_LANGUAGE"] == 'Chinese':
                        box = self.client.data_process(box, w_, h_)
                    else:
                        box = [int(box[0] * w_), int(box[1] * h_), int(box[2] * w_), int(box[3] * h_)]
                box = [box[0], box[1], box[2] - box[0], box[3] - box[1]]
                self.track.set_track_target(box, image)
                self.start_track = True
                speech.play_audio(start_track_audio_path, block=False)
            except (ValueError, TypeError) as e:
                self.start_track = False
                msg = String()
                msg.data = self.vllm_result
                self.tts_text_pub.publish(msg)
                speech.play_audio(track_fail_audio_path, block=False)
                self.get_logger().info(e)
            self.vllm_result = ''
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)           
        if self.start_track:
            self.data = self.track.track(image, depth_image)
            image = self.data[-1]
            twist = Twist()
            twist.linear.x, twist.angular.z = self.data[0], self.data[1]
            # self.get_logger().info('twist.linear.x:{}'.format(twist.linear.x))
            # self.get_logger().info('twist.angular.z:{}'.format(twist.angular.z))                                

            if 'Acker' in self.machine_type:
                steering_angle = common.set_range(twist.angular.z, -math.radians(40), math.radians(40))
                if steering_angle != 0:
                    R = 0.145/math.tan(steering_angle)
                    twist.angular.z = twist.linear.x/R


            self.mecanum_pub.publish(twist)                             
        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        self.fps.update()
        self.fps.show_fps(image)
        cv2.imshow('image', image)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # Press q or ESC to exit
            self.mecanum_pub.publish(Twist())
            self.running = False
```

By continuously retrieving RGB and depth image frames from the image queue, when the visual language model output is detected, the target bounding box coordinates are extracted. After coordinate transformation based on the camera type and language environment, tracking is initiated, and the start prompt sound is played. If parsing fails, a voice feedback is provided via TTS. After performing a kinematic transformation based on the robot type, the control commands are published to the chassis control topic for chassis control, with support for exiting the loop via a button press.

5\. `multi_callback` Method

```python
def multi_callback(self, depth_image, ros_image):
    depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)
    bgr_image = np.array(self.bridge.imgmsg_to_cv2(ros_image, "bgr8"), dtype=np.uint8)

    if self.image_queue.full():
        # If the queue is full, discard the oldest image
        self.image_queue.get()
    # Put the image into the queue
    self.image_queue.put([bgr_image, depth_frame])
```

Convert the ROS depth image message into a depth frame in `NumPy` array format, while using `CVBridge` to convert the ROS RGB image message into an `OpenCV` BGR format image. If the image queue is full, first clear the queue, and then insert the processed BGR image and depth frame into the image queue.

### 19.4.4 Smart Home Assistant

The large model used in this lesson operates online, requiring a stable network connection for the controller during the operation.

#### 19.4.4.1 Program Overview

When the program starts, the voice device will announce “I'm ready.” Then, speak the wake word **Hello Hiwonder** to activate the voice device, which will respond with **I'm here**. After activation, voice commands can be used to control the robot. For example, issuing the instruction:  
**Go to the kitchen to see if the door is closed, then come back and tell me**. Upon receiving a command, the terminal displays the recognized speech content. The voice device then verbally responds with a generated answer, and the robot simultaneously executes the corresponding action.

#### 19.4.4.2 Preparation

* **Version Confirmation**

Before starting this feature, ensure that the microphone version configuration in the system is correct. For more details, refer to section [19.3.2 Version Confirmation](#anther19.3.2).

* **Configure Large Model API-KEY**

Refer to the section [19.3.1 Large Model API Key Setup](#anther19.3.1) to set up the large model key.

* **Navigation Map Construction**

Before enabling this feature, a map must be created in advance. Please refer to [14.1 Mapping Tutorial](https://wiki.hiwonder.com/projects/ROSOrin/en/jetson-nano/docs/14_ROS2_Mapping_%26_Navigation_Course.html#mapping-tutorial) in the file **14. ROS2-Mapping \& Navigation Course** for detailed instructions on how to build the map.

#### 19.4.4.3 Operation Steps

> [!NOTE]
>
> **Command input is case-sensitive and space-sensitive.**
>
> **The robot must be connected to the Internet, either in STA (LAN) mode or AP (direct connection) mode via Ethernet.**

1. Click the terminal <img  src="../_static/media/chapter_19/section_4/media/image25.png"  /> on the left side of the system interface to open the command line. Enter the command and press **Enter** to disable the app auto-start service.

```bash
sudo systemctl stop start_app_node.service
```

2) Open the command line terminal <img src="../_static/media/chapter_19/section_4/media/image14.png"   /> from the left side of the system interface. In the terminal window, enter the following command and press **Enter** to launch the smart home assistant feature.

```bash
ros2 launch large_models_examples vllm_navigation.launch.py map:=map_01
```

3) When the command line displays the following output and announces **I'm ready**, it indicates that the voice device has completed initialization.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image38.png" style="width:700px" />

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image39.png" style="width:700px" />

4) Say the wake word **Hello Hiwonder**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image40.png" style="width:700px" />

5) The voice device will then respond with **I'm here**, signaling that it has been activated successfully. The system will begin recording voice commands.

6) When the command line displays the following output, it indicates that the voice device has printed the recognized speech. At this point, the system begins recording the received command. Then, give the command to the robot, for example, **Go to the kitchen to see if the door is closed, then come back and tell me**, and wait for the large model to process the command. When the command line shows the next output, it indicates that the cloud-based speech large model has successfully processed the voice command and parsed the audio. The parsed result will appear in the **publish_asr_result**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image42.png" style="width:700px" />

7. When the command line displays the output shown below, the cloud-based large language model has been successfully invoked. It processes the voice command, provides a verbal response, and generates actions corresponding to the semantic meaning of the command.

   The response is automatically generated by the large model. Only the semantic accuracy of the reply is guaranteed, while the wording and formatting may vary.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image43.png" style="width:700px" />

8) Upon reaching the **kitchen**, the system will recognize whether the door is closed.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image44.png" style="width:700px" />

9) Afterward, the robot will return to the **origin** position and provide a response about the door's status, such as **The door is closed**.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image40.png" style="width:700px" />

10) When the terminal shows the output shown in the figure, indicating the end of one interaction cycle, the system is ready for the next round. To initiate another interaction, repeat step 5 by speaking the wake words again.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image35.png" style="width:700px" />

11) To exit the feature, press **Ctrl+C** in the terminal. If the feature does not exit immediately, press **Ctrl+C** multiple times.

#### 19.4.4.4 Program Outcome

Once the feature starts, any command can be issued to the robot, such as **Go to the kitchen to see if the door is closed and let me know when you come back**. The robot will navigate to the designated location, check if the door is closed, and then return to the starting point to inform of the result.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image40.png" style="width:700px" />

#### 19.4.4.5 Modifying Navigation Locations

To modify the navigation positions in the program, edit the file located at the following path:

**~/ros2_ws/src/large_models_examples/large_models_examples/vllm_navigation.py.**

1) Open the program to display the RViz map, then set the navigation target location by clicking `2D Goal Pose` and selecting the position on the map.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image45.png" style="width:700px" />

2. Return to the command terminal and check the published target position parameters.

   <img class="common_img" src="../_static/media/chapter_19/section_4/media/image46.png" style="width:700px"  />


Locate the corresponding section of the code shown below, and fill in the target location parameters after the appropriate location name.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image47.png" style="width:700px" />

The different target navigation points in the program are relative to the robot's starting point during the mapping process. Each navigation point includes five parameters:

x: position on the x-axis (meters)

y: position on the y-axis (meters)

roll: rotation around the x-axis (degrees)

pitch: rotation around the y-axis (degrees)

yaw: rotation around the z-axis (degrees)

For example, to define the position of **kitchen**, the parameters can be set to [1.45, -0.29, 0.0, 0.0, 0.0], indicating that the robot will navigate to a location offset from the map origin by those values.

The robot's target position orientation is updated. The printed quaternion `Orientation(0, 0, -0.379167, 0.925328)= Angle: -0.777791` in radians is shown. The quaternion is then converted to Euler angles, `roll, pitch, yaw`, resulting in `roll ≈ 0°，pitch ≈ 0°，yaw ≈ −44.56°`. Alternatively, converting the radians directly to degrees gives `−44.58°`.

<img class="common_img" src="../_static/media/chapter_19/section_4/media/image48.png" style="width:700px" />

Therefore, the final modified position for **kitchen** should be set as [1.45, -0.29, 0.0, 0.0, −44.58].

#### 19.4.4.6 Program Analysis

* **Launch File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/vllm_navigation.launch.py**.

Define `launch_setup` Function

```python
def launch_setup(context):
    slam_package_path = get_package_share_directory('slam')
    navigation_package_path = get_package_share_directory('navigation')
    large_models_package_path = get_package_share_directory('large_models')
    
    mode = LaunchConfiguration('mode', default=1)
    mode_arg = DeclareLaunchArgument('mode', default_value=mode)
    map_name = LaunchConfiguration('map', default='map_01').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST'])
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER'])

    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/navigation.launch.py')),
        launch_arguments={
            'sim': 'false',
            'map': map_name,
            'robot_name': robot_name,
            'master_name': master_name,
            'use_teb': 'true',
        }.items(),
    )

    navigation_controller_node = Node(
        package='large_models_examples',
        executable='navigation_controller',
        output='screen',
        parameters=[{'map_frame': 'map', 'nav_goal': '/nav_goal'}]
    )

    rviz_node = ExecuteProcess(
            cmd=['rviz2', 'rviz2', '-d', os.path.join(navigation_package_path, 'rviz/navigation_controller.rviz')],
            output='screen'
        )

    large_models_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(large_models_package_path, 'launch/start.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    vllm_navigation_node = Node(
        package='large_models_examples',
        executable='vllm_navigation',
        output='screen',
    )
```

`launch_setup`: Used to configure and initiate the `ROS 2 Launch` file actions.

`node = LaunchConfiguration('node', default=1)`: Defines a launch parameter named node, with a default value of 1.

`navigation_package_path`: Retrieves the shared directory path of the navigation package `parigations`.

`large_model_package_path`: Retrieves the shared directory path of the large model package `large_model`.

`navigating_lunch`: Uses `IncludeLaunchDescription` to include the navigation launch file `navigation.launch.py`, passing parameters like `node_name` and `master_name`.

`vllm_navigation_node`: Defines a ROS 2 node from the `large_models_examples` package, executing the `vllm_navigation` executable, and outputting to the screen.

`navigating_controller_node`: Defines the navigation control node from the `large_models_examples` package.

* **Python File Analysis**

The program file is located at: **/home/ubuntu/ros2_ws/src/large_models_examples/large_models_examples/vllm_navigation.py**.

1\. Define Prompt Template (PROMPT)

```python
    LLM_PROMPT = '''
**Role
You are a smart navigation vehicle equipped with a camera and speaker. You can move to different places, analyze visual input, and respond by playing audio. Based on user input, you need to generate the corresponding JSON command.

**Requirements
- For any user input, look up corresponding functions from the Action Function Library, and generate the proper JSON output.
- For each action sequence, include a concise (5–20 characters) and witty, varied response to make the interaction lively and engaging.
- Output only the JSON result, no analysis or extra text.
- Output format:
{
  "action": ["xx", "xx"],
  "response": "xx"
}

**Special Notes
The "action" field contains an ordered list of function names to be executed in sequence. If no matching function is found, return: "action": [].
The "response" field should contain a carefully crafted, short, humorous, and varied message (5–20 characters).

**Action Function Library
Move to a specified place: move('kitchen')
Return to starting point: move('origin')
Analyze current view: vision('What do you see?')
Play audio response: play_audio()

**Example
Input: Go to the front desk to see if the door is closed, and then come back and tell me
Output:
{
  "action": ["move('front desk')", "vision('Is the door closed?')", "move("origin")", "play_audio()"],
  "response": "On it, reporting soon!"
}
    '''
```

2\. Class Initialization

```python
def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.response_text = ''
        self.llm_result = ''
        self.play_audio_finish = False
        self.running = True
        self.play_delay = False
        self.reach_goal = False
        self.interrupt = False
        
        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        # self.create_subscription(Image, 'ascamera/camera_publisher/rgb0/image', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(Bool, 'navigation_controller/reach_goal', self.reach_goal_callback, 1)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.set_vllm_content_client = self.create_client(SetContent, 'agent_process/set_vllm_content')
        self.set_vllm_content_client.wait_for_service()
        self.set_pose_client = self.create_client(SetPose2D, 'navigation_controller/set_pose')
        self.set_pose_client.wait_for_service()

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/arm_pc/ActionGroups')

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
```

Set up state parameters, including action list, response text, LLM processing results, and navigation target arrival status. Create publishers for TTS text publishing and subscribers for receiving agent processing results, voice wake-up signals, audio playback completion status, and navigation arrival status. At the same time, establish service client requests for voice wake-up, node settings, prompt word configuration, visual content settings, and navigation target point setup. Finally, create a timer to initiate the initialization process.

3\. `init_process` Method

```python
def init_process(self):
    self.timer.cancel()

    msg = SetModel.Request()
    msg.model = llm_model
    msg.model_type = 'llm'
    msg.api_key = api_key 
    msg.base_url = base_url
    self.send_request(self.set_model_client, msg)

    msg = SetString.Request()
    msg.data = LLM_PROMPT
    self.send_request(self.set_prompt_client, msg)

    init_finish = self.create_client(Empty, 'navigation_controller/init_finish')
    init_finish.wait_for_service()
    speech.play_audio(start_audio_path)
    threading.Thread(target=self.process, daemon=True).start()
    self.create_service(Empty, '~/init_finish', self.get_node_state)
    self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    self.get_logger().info('\033[1;32m%s\033[0m' % LLM_PROMPT)
```

After disabling the timer, configure the parameters for the large language model, including model type, API key, and base URL. Set up a dedicated prompt word template for the navigation task, wait for the navigation control system to initialize, and play the start prompt audio. At the same time, start the processing thread, create a service for the initialization completion status, and finally output the log information to indicate that the system is ready.

4\. `move` Method

```python
def move(self, position):
    self.get_logger().info('position: %s' % str(position))
    msg = SetPose2D.Request()
    if position not in position_dict:
        return False
    p = position_dict[position]
    msg.data.x = float(p[0])
    msg.data.y = float(p[1])
    msg.data.roll = p[2]
    msg.data.pitch = p[3]
    msg.data.yaw = p[4]
    self.send_request(self.set_pose_client, msg)
    return True
```

By receiving position parameters, query the corresponding coordinates and orientation information, including `x`, `y` coordinates and `roll`, `pitch`, `yaw` rotation angles, from the preset location dictionary. Create a `SetPose2D` type service request message, populate the data, and then send it to the navigation server, enabling the robot to navigate to the specified preset location.

5\. `reach_goal_callback` Method

```python
def reach_goal_callback(self, msg):
    self.get_logger().info('reach goal')
    self.reach_goal = msg.data
```

When the goal arrival status message published by the navigation controller is received, assign the boolean value from the message to the node's `reach_goal` status flag and log the `reach_goal` information. This ensures synchronization of the navigation system's state update and facilitates the subsequent task decision process.

6\. `vision` Method

```python
def vision(self, query):
    self.controller.run_action('horizontal')
    msg = SetContent.Request()
    if language == 'Chinese':
        msg.api_key = stepfun_api_key
        msg.base_url = stepfun_base_url
        msg.model = stepfun_vllm_model
    else:
        msg.api_key = vllm_api_key
        msg.base_url = vllm_base_url
        msg.model = vllm_model
    msg.prompt = VLLM_PROMPT
    msg.query = query
    self.get_logger().info('vision: %s' % query)
    res = self.send_request(self.set_vllm_content_client, msg)
    self.controller.run_action('init')
    return res.message
```

By receiving the query parameters, the system selects the appropriate visual language model configuration based on the language setting, sets the dedicated visual prompt template and query content, then sends a request to the visual language content service and returns the model’s response message.

7\. `process` Method

```python
def process(self):
    first = True
    while self.running:
        if self.llm_result:
            self.interrupt = False
            msg = String()
            if 'action' in self.llm_result: # If a corresponding action is returned, extract and process it
                result = json.loads(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}')+1])
                if 'response' in result:
                    msg.data = result['response']
                    self.tts_text_pub.publish(msg)
                if 'action' in result:
                    action = result['action']
                    self.get_logger().info(f'vllm action: {action}')
                    for a in action:
                        if 'move' in a:
                            self.reach_goal = False
                            res = eval(f'self.{a}')
                            if res:
                                while not self.reach_goal:
                                    if self.interrupt:
                                        self.get_logger().info('interrupt')
                                        break
                                    # self.get_logger().info('waiting for reach goal')
                                    time.sleep(0.01)
                        elif 'vision' in a:
                            res = eval(f'self.{a}')
                            self.response_text = res
                            self.get_logger().info(f'vllm response: {res}')
                            self.play_audio()
                        elif 'play_audio' in a:
                            eval(f'self.{a}')
                            while not self.play_audio_finish:
                                time.sleep(1)
                        if self.interrupt:
                            self.get_logger().info('interrupt')
                            break
            else: # If there is no corresponding action, only respond
                msg.data = self.llm_result
                self.tts_text_pub.publish(msg)
            self.action_finish = True
            self.llm_result = ''
        else:
            time.sleep(0.01)
        if self.play_audio_finish and self.action_finish:
            self.play_audio_finish = False
            self.action_finish = False
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)
    rclpy.shutdown()
```

By continuously monitoring the output of the large model, when action instructions are detected, the action list and response text are parsed. If there is a movement command, navigation is executed, and the system waits for the target point to be reached while supporting interruption. If there is a visual command, a visual query is executed and the result is recorded. If there is an audio command, the corresponding content is played, and the text response is sent to the TTS module for speech output. If only a text response is provided, it will be relayed as speech feedback. During the process, coordination between action execution and the completion status of speech playback is managed. Once the task is completed, the voice wake-up function is re-enabled, and the system waits for the next command.
