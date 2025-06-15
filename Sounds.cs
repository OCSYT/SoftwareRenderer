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
            public volatile bool ShouldStop;
        }

        private static readonly ConcurrentDictionary<int, SoundInstance> PlayingSounds = new();
        private static int _nextSoundId = 1;
        private static readonly object _idLock = new();
        private static bool _sdlInitialized = false;
        private static readonly object _initLock = new();

        private unsafe static void AdjustVolume(IntPtr buffer, uint length, float volume)
        {
            if (buffer == IntPtr.Zero || length == 0) return;
            
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

        private static bool EnsureSDLInitialized()
        {
            lock (_initLock)
            {
                if (!_sdlInitialized)
                {
                    if (SDL.SDL_Init(SDL.SDL_INIT_AUDIO) != 0)
                    {
                        Console.WriteLine("SDL_Init Error: " + SDL.SDL_GetError());
                        return false;
                    }
                    _sdlInitialized = true;
                }
                return true;
            }
        }

        // Returns a unique sound ID you can use to stop this sound later.
        public static int PlaySound(string filePath, float volume, bool loop = false)
        {
            if (string.IsNullOrEmpty(filePath))
            {
                Console.WriteLine("Invalid file path");
                return -1;
            }

            try
            {
                if (!EnsureSDLInitialized())
                {
                    return -1;
                }

                if (SDL.SDL_LoadWAV(filePath, out var spec, out var audioBuf, out var audioLen) == IntPtr.Zero)
                {
                    Console.WriteLine("Could not load WAV file: " + filePath + " - " + SDL.SDL_GetError());
                    return -1;
                }

                // Validate audio data
                if (audioBuf == IntPtr.Zero || audioLen == 0)
                {
                    Console.WriteLine("Invalid audio data loaded");
                    if (audioBuf != IntPtr.Zero)
                        SDL.SDL_FreeWAV(audioBuf);
                    return -1;
                }

                AdjustVolume(audioBuf, audioLen, volume);

                spec.callback = null;
                spec.userdata = IntPtr.Zero;

                uint deviceId = SDL.SDL_OpenAudioDevice(null, 0, ref spec, out _, 0);
                if (deviceId == 0)
                {
                    Console.WriteLine("Failed to open audio device: " + SDL.SDL_GetError());
                    SDL.SDL_FreeWAV(audioBuf);
                    return -1;
                }

                // Validate device opened successfully
                if (SDL.SDL_QueueAudio(deviceId, audioBuf, audioLen) != 0)
                {
                    Console.WriteLine("Failed to queue audio: " + SDL.SDL_GetError());
                    SDL.SDL_CloseAudioDevice(deviceId);
                    SDL.SDL_FreeWAV(audioBuf);
                    return -1;
                }

                SDL.SDL_PauseAudioDevice(deviceId, 0);

                int soundId;
                lock (_idLock)
                {
                    soundId = _nextSoundId++;
                }

                var soundInstance = new SoundInstance
                {
                    DeviceId = deviceId,
                    AudioBuf = audioBuf,
                    AudioLen = audioLen,
                    ShouldStop = false
                };

                PlayingSounds[soundId] = soundInstance;

                _ = Task.Run(async () =>
                {
                    try
                    {
                        // Validate audio format for duration calculation
                        int bytesPerSample = SDL.SDL_AUDIO_BITSIZE(spec.format) / 8;
                        if (bytesPerSample <= 0 || spec.freq <= 0 || spec.channels <= 0)
                        {
                            Console.WriteLine("Invalid audio format parameters");
                            return;
                        }

                        int bytesPerSecond = spec.freq * bytesPerSample * spec.channels;
                        int durationMs = Math.Max(1, (int)((audioLen / (float)bytesPerSecond) * 1000));

                        if (loop)
                        {
                            while (!soundInstance.ShouldStop && PlayingSounds.ContainsKey(soundId))
                            {
                                await Task.Delay(durationMs);
                                
                                if (soundInstance.ShouldStop || !PlayingSounds.ContainsKey(soundId))
                                    break;

                                // Check if device is still valid before queuing
                                var deviceStatus = SDL.SDL_GetAudioDeviceStatus(deviceId);
                                if (deviceStatus != SDL.SDL_AudioStatus.SDL_AUDIO_STOPPED)
                                {
                                    SDL.SDL_ClearQueuedAudio(deviceId);
                                    if (SDL.SDL_QueueAudio(deviceId, audioBuf, audioLen) != 0)
                                    {
                                        Console.WriteLine("Failed to re-queue audio for loop: " + SDL.SDL_GetError());
                                        break;
                                    }
                                }
                                else
                                {
                                    break; // Device is stopped/invalid
                                }
                            }
                        }
                        else
                        {
                            await Task.Delay(durationMs);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"Error in audio playback task: {ex.Message}");
                    }
                    finally
                    {
                        // Clean up resources
                        if (PlayingSounds.TryRemove(soundId, out var sound))
                        {
                            sound.ShouldStop = true;
                            
                            try
                            {
                                if (sound.DeviceId != 0)
                                {
                                    SDL.SDL_CloseAudioDevice(sound.DeviceId);
                                }
                                if (sound.AudioBuf != IntPtr.Zero)
                                {
                                    SDL.SDL_FreeWAV(sound.AudioBuf);
                                }
                            }
                            catch (Exception ex)
                            {
                                Console.WriteLine($"Error cleaning up audio resources: {ex.Message}");
                            }
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
            if (PlayingSounds.TryGetValue(soundId, out var sound))
            {
                sound.ShouldStop = true;
            }

            if (PlayingSounds.TryRemove(soundId, out sound))
            {
                try
                {
                    if (sound.DeviceId != 0)
                    {
                        SDL.SDL_ClearQueuedAudio(sound.DeviceId);
                        SDL.SDL_CloseAudioDevice(sound.DeviceId);
                    }
                    if (sound.AudioBuf != IntPtr.Zero)
                    {
                        SDL.SDL_FreeWAV(sound.AudioBuf);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error stopping sound: {ex.Message}");
                }
            }
        }

        public static void StopAllSounds()
        {
            var soundIds = new int[PlayingSounds.Count];
            PlayingSounds.Keys.CopyTo(soundIds, 0);
            
            foreach (var soundId in soundIds)
            {
                StopSound(soundId);
            }
        }

        public static void Cleanup()
        {
            StopAllSounds();
            
            lock (_initLock)
            {
                if (_sdlInitialized)
                {
                    SDL.SDL_QuitSubSystem(SDL.SDL_INIT_AUDIO);
                    _sdlInitialized = false;
                }
            }
        }
    }
}