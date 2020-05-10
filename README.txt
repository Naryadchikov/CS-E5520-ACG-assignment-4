# CS-E5520 Advanced Computer Graphics, Spring 2020
# Lehtinen / Aarnio, Kemppinen
#
# Assignment 4: Path Tracing

Student name: Nariadchikov Aleksandr
Student number: 843513

# Which parts of the assignment did you complete? Mark them 'done'.
# You can also mark non-completed parts as 'attempted' if you spent a fair amount of
# effort on them. If you do, explain the work you did in the problems/bugs section
# and leave your 'attempt' code in place (commented out if necessary) so we can see it.

R1 Integrate your raytracer (1p): done
  R2 Direct light & shadows (2p): done
  R3 Bounced indirect light (5p): done
        R4 Russian roulette (2p): done

Remember the other mandatory deliverables!
- 6 images rendered with our default presets
- Rendering competition PDF + images in separate folder -> I will submit competition entry next week!



# Did you do any extra credit work?
(Describe what you did and, if there was a substantial amount of work involved, how you did it. If your extra features are interactive or can be toggled, describe how to use them.)

Overall, I've tried to document code as full as possible, so if some explanations are missing here, you will find them in source code.

!!!For better navigation, I've added [Controls] tag, where I've added new slider or toggle.
Example:
[Controls] New slider for choosing termination probability for Russian roulette.

EXTRA FEATURES:

- Support for light-emitting triangles
[Controls] New toggle whether or not use emissive triangles during computations.
I've done sampling according to the emissive power of triangles and present light sources on scene.


- Tangent space normal mapping
For understanding how to do it I've read the following article:
http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-13-normal-mapping/


- Glossy reflection model (Blinn-Phong)
Probably, it was the hardest part of the assignment for me.
I am still not handling reflection direction in the right way (More details about it I will write later, as well as in 'Reflections and Refractions' section).

Firstly, I've read several articles about importance sampling and implemented it:
http://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/Importance_Sampling.html
https://computergraphics.stackexchange.com/questions/4979/what-is-importance-sampling

Then I gathered a general understanding of Blinn-Phong model:
https://paroj.github.io/gltut/Illumination/Tut11%20BlinnPhong%20Model.html
https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_reflection_model

Added specular part into account if shadow ray isn't blocked.
Firstly, I am doing Blinn-Phong Normalization:
http://www.thetenthplanet.de/archives/255

Then calculating half vector between the light vector and ray direction.
Then adding specular part to color as: (dot(HalfVector, surfaceNormal))^glossiness * normalizationFactor * specular;

For computing reflection direction I was trying to use half vector as: ref_dir = ray_dir + 2 * dot(ray_dir, HalfVector) * HalfVector;
But it hasn't produced a proper image.

Then I've implemented pure mirror reflection, it is working right.
For debug I've added a toggle:
[Controls] New toggle, should pure mirror reflection be used or not (default: true).

Another problem appeared when I am adding components to throughoutput.
As I understand after Pauili explanation, in general case I should do like that:
throughput *= cos(theta) * (diffuse + specular) / pdf;

However it is not working right, so as a default I am using only diffuse part:
throughput *= diffuse; (cos(theta) * (diffuse / Pi) / (cos(theta) / Pi))

Here new toggle is added:
[Controls] New toggle, should be accounting only diffuse part to throughput or not (default: true).

I would be really grateful if you explain to me what I am doing wrong in computing throughput and (glossy) reflection direction.

P.S. I've added few screenshots with reflections into assignment folder.


- Properly uncorrelated QMC samples (Sobol sequence)
AA rays - 1st and 2nd dimensions;
1st bounce draws from 3rd and 4th dimensions;
2nd bounce gets dimensions 5th and 6th;
and so on.

For better understanding about Anti-aliasing I've looked at the following slides:
https://courses.cs.washington.edu/courses/cse457/17au/assets/lectures/aa-and-mcpt-1pp.pdf


- Debug visualizer
I've done a simple implementation.
It shows the path segments (white for camera/bounced rays; blue for Russian roulette rays).
As well as path segments it shows Ei for each hit point, hit normals (red), shadow rays (yellow) and normals on lights (red).


- Several area light sources on scene
[Controls] '+' for adding new light on scene.
[Controls] '-' for removing selected light.
[Controls] 'Shift' for changing controlled light (carousel style).
[Controls] There are 3 new sliders for selecting color of the controlled light between [0, 255] values.
[Controls] There is a new slider for changing controlled light intensity.
!!! Emission vector for area lights is computing like that now: m_E = intensity * Vec3f(R, G, B).normalized();
For choosing which light source to sample I am looking at emissive power.
One with the largest emissive power would be chosen for final sampling.

Screenshots of 'conference-new' with 2 lamps are added in screenshot folder.


- Reflections and Refractions
Firstly, if you want to see reflections and refractions use that toggle:
[Controls] New toggle, should reflection and refraction rays be present (default: false).

If perfect mirror reflections are not used, 
then I've tried to fake glossy reflections by adding 0.1f * cwd part to reflection ray. (it is not right, I know... But how should I do?)

For gather some information and make pure reflections and refraction I've read 8th chapter of PBR book:
http://www.pbr-book.org/3ed-2018/Reflection_Models.html


- Gauss filter
[Controls] New slider for controlling filter width.

General understanding:
https://en.wikipedia.org/wiki/Gaussian_blur

For gathering some common information about denoising technics, I've read with some cross-links:
https://alain.xyz/blog/raytracing-denoising



# Have you done extra credit work on previous rounds whose grading we have postponed to this round?
(Are all the features integrated into your Assn4 submission? If not, point out which one of your submissions we should use to grade those features.)

- Quasi Monte Carlo sampling (was mentioned in assignment 2 pdf file)
I've done QMC sampling using the Sobol sequence:
Light sampling and hemisphere sampling in indirect bounces.

For a primary understanding of how Sobol sequence is looked like I've gone through:
https://en.wikipedia.org/wiki/Sobol_sequence

For source files I've used:
http://gruenschloss.org/



# Are there any known problems/bugs remaining in your code?
(Please provide a list of the problems. If possible, describe what you think the cause is, how you have attempted to diagnose or fix the problem, and how you would attempt to diagnose or fix it if you had more time or motivation. This is important: we are more likely to assign partial credit if you help us understand what's going on.)

As for general the part, there are no issues.
Problems with glossy reflections I've discussed in 'extra credit work' section and in comments of the source code.


!!! However, during the work process I've accidentally found out some weird thing:
If during sampling of a cosine-weighted hemisphere sometimes passing negative numbers to square root function, getting NaN value and terminating path iteration,
you are getting better results in most cases, compared with right implementation...

Firstly, it gives you a significant speed boost (you are skipping a big amount of paths).
Secondly, it makes picture with less noise...

Probably, I am going crazy because I am sitting on that assignment for more than 1 week (8+ hours per day), but still...
I've decided to add that weird thing in assignment source code, so you can check it out.
I've added a button to turn that on:
[Controls] Magic Button - Adding pure magic to the image (joking, but the result is still magical).

As well I've added few screenshots for comparison in screenshot folder with time stamps.
I hope, you could comment on that.



# Did you collaborate with anyone in the class?
(Did you help others? Did others help you? Let us know who you talked to, and what sort of help you gave or received.)

Pauli helped me a lot by sharing with useful papers and explanations of sampling in general.
And he helped me at the beginning with finding a bug with textures: I was using wrong order of barycentrics for vertices.

Also, I asked my friend, who is a rendering programmer in game development, to make a small lecture about lighting models.
Because I haven't had computer graphics intro class (I am an exchange student here), I have some spaces in theory.
So we used HLSL shaders to go through some lighting fundamentals: direct, ambient, specular and etc.



# Any other comments you'd like to share about the assignment or the course overall?
(Was the assignment too long? Too hard? Fun or boring? Did you learn something, or was it a total waste of time? Can we do something differently to help you learn? Please be brutally honest; we won't take it personally.)

First of all, thank you for reading that "long read" readMe file!
And thank you for the course in general, I would say that it is my most favourite course, that I took here as an exchange student.

However, after our lectures moved to 'Zoom' it is started to be way harder to understand the content and concentrate...
Maybe it is my personal feeling, but I decided to share with you.

Because of not catching up with Zoom lecture content sometimes, I was filling the gaps with other resources:

I've watched the following course on YouTube(it was quite helpful):
TU Wien Rendering Course (https://www.youtube.com/watch?v=pjc1QAI6zS0&list=PLujxSBD-JXgnGmsn7gEyN28P1DnRZG7qi)

I've read some chapters of PBR book (http://www.pbr-book.org/3ed-2018/contents.html)
Probably, will start it from the beginning someday, doing all exercises.

And, finally, I searched a lot of things on "Computer Graphics Stackexchange" site.


Overall, I want to continue digging deeper into Computer Graphics!
I think it would be quite useful for me, eventhough I am mostly focused on game design and gameplay programming for the last years.


P.S. I will use an extra week for trying to make proper glossy model (hoping that you could provide a feedback on present mistakes) and to make a nice scene for competition entry!