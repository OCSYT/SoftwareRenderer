using SDL2;
using System;
using System.Collections.Concurrent;
using System.Threading.Tasks;

namespace SoftwareRenderer
{
    public static class Sounds
    {
        private class SoundInstance
        {
            public uint DeviceId;
            public IntPtr AudioBuf;
            public uint AudioLen;
        }

        private static readonly ConcurrentDictionary<int, SoundInstance> PlayingSounds = new();
        private static int _nextSoundId = 1;
        private static readonly object _idLock = new();

        private unsafe static void AdjustVolume(IntPtr buffer, uint length, float volume)
        {
            short* samples = (short*)buffer;
            int sampleCount = (int)(length / 2);
            for (int i = 0; i < sampleCount; i++)
            {
                int sample = (int)(samples[i] * volume);
                if (sample > short.MaxValue) sample = short.MaxValue;
                if (sample < short.MinValue) sample = short.MinValue;
                samples[i] = (short)sample;
            }
        }

        // Returns a unique sound ID you can use to stop this sound later.
        public static int PlaySound(string filePath, float volume, bool loop = false)
        {
            try
            {
                if (SDL.SDL_Init(SDL.SDL_INIT_AUDIO) != 0)
                {
                    Console.WriteLine("SDL_Init Error: " + SDL.SDL_GetError());
                    return -1;
                }

                if (SDL.SDL_LoadWAV(filePath, out var spec, out var audioBuf, out var audioLen) == IntPtr.Zero)
                {
                    Console.WriteLine("Could not load WAV file: " + filePath);
                    return -1;
                }

                AdjustVolume(audioBuf, audioLen, volume);

                spec.callback = null;
                spec.userdata = IntPtr.Zero;

                uint deviceId = SDL.SDL_OpenAudioDevice(null, 0, ref spec, out _, 0);
                if (deviceId == 0)
                {
                    Console.WriteLine("Failed to open audio device.");
                    SDL.SDL_FreeWAV(audioBuf);
                    return -1;
                }

                SDL.SDL_QueueAudio(deviceId, audioBuf, audioLen);
                SDL.SDL_PauseAudioDevice(deviceId, 0);

                int soundId;
                lock (_idLock)
                {
                    soundId = _nextSoundId++;
                }

                PlayingSounds[soundId] = new SoundInstance
                {
                    DeviceId = deviceId,
                    AudioBuf = audioBuf,
                    AudioLen = audioLen
                };

                _ = Task.Run(async () =>
                {
                    try
                    {
                        int bytesPerSample = SDL.SDL_AUDIO_BITSIZE(spec.format) / 8;
                        int bytesPerSecond = spec.freq * bytesPerSample * spec.channels;
                        int durationMs = (int)((audioLen / (float)bytesPerSecond) * 1000);

                        while (loop)
                        {
                            await Task.Delay(durationMs);
                            if (!PlayingSounds.ContainsKey(soundId)) break;

                            SDL.SDL_ClearQueuedAudio(deviceId);
                            SDL.SDL_QueueAudio(deviceId, audioBuf, audioLen);
                        }

                        if (!loop)
                            await Task.Delay(durationMs);
                    }
                    finally
                    {
                        if (PlayingSounds.TryRemove(soundId, out var sound))
                        {
                            SDL.SDL_CloseAudioDevice(sound.DeviceId);
                            SDL.SDL_FreeWAV(sound.AudioBuf);
                        }
                    }
                });

                return soundId;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Failed to play sound: {ex.Message}");
                return -1;
            }
        }

        public static void StopSound(int soundId)
        {
            if (PlayingSounds.TryRemove(soundId, out var sound))
            {
                SDL.SDL_ClearQueuedAudio(sound.DeviceId);
                SDL.SDL_CloseAudioDevice(sound.DeviceId);
                SDL.SDL_FreeWAV(sound.AudioBuf);
            }
        }
    }
}