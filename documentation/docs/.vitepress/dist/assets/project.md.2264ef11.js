import{_ as e,o,c as t,V as s}from"./chunks/framework.5c100d06.js";const a="/open-source-autonomous-vehicle-controller/assets/connection.5cc0f7fc.jpg",l="/open-source-autonomous-vehicle-controller/assets/usbSerial.68e93808.png",r="/open-source-autonomous-vehicle-controller/assets/baudRate.734f7257.png",n="/open-source-autonomous-vehicle-controller/assets/openProject.7d915e17.png",i="/open-source-autonomous-vehicle-controller/assets/serial.bf94765a.png",p="/open-source-autonomous-vehicle-controller/assets/projectProperties.49c3af93.png",c="/open-source-autonomous-vehicle-controller/assets/xc32id.1fb79a82.png",u="/open-source-autonomous-vehicle-controller/assets/general.53d42caf.png",h="/open-source-autonomous-vehicle-controller/assets/bytes.378e6cfd.png",m="/open-source-autonomous-vehicle-controller/assets/ok.55d84c34.jpg",d="/open-source-autonomous-vehicle-controller/assets/CleanBuild.65f4ce2d.png",g="/open-source-autonomous-vehicle-controller/assets/MakeProgram.d9e42909.png",b="/open-source-autonomous-vehicle-controller/assets/outputCoolterm.1894a0b6.png",C="/open-source-autonomous-vehicle-controller/assets/everything.9ad8dd08.png",D=JSON.parse('{"title":"Building Your First Project","description":"","frontmatter":{},"headers":[],"relativePath":"project.md","filePath":"project.md"}'),y={name:"project.md"},_=s('<h1 id="building-your-first-project" tabindex="-1">Building Your First Project <a class="header-anchor" href="#building-your-first-project" aria-label="Permalink to &quot;Building Your First Project&quot;">​</a></h1><h2 id="cloning-the-repository" tabindex="-1">Cloning the Repository <a class="header-anchor" href="#cloning-the-repository" aria-label="Permalink to &quot;Cloning the Repository&quot;">​</a></h2><p>To clone the Repo just execute the following command on Gitbash or your terminal prompt. We are making the project folder in Desktop you can clone it into directory you want. Navigate to your desired directory</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#82AAFF;">cd</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">Desktop</span></span></code></pre></div><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">clone</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">https://github.com/uccross/open-source-autonomous-vehicle-controller.git</span></span></code></pre></div><p>Now navigate to the modules directory in the Cloned Repo</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#82AAFF;">cd</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">open-source-autonomous-vehicle-controller/modules</span></span></code></pre></div><p>Clone the submodules in this directory</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">clone</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">https://github.com/mavlink/c_library_v2</span></span></code></pre></div><h2 id="setting-up-hardware" tabindex="-1">Setting up Hardware <a class="header-anchor" href="#setting-up-hardware" aria-label="Permalink to &quot;Setting up Hardware&quot;">​</a></h2><p><img src="'+a+'" alt="Connection"></p><ul><li>Connect ICSP (in circuit serial programming) of PICKIT 3 to the OSAVC <ul><li>Use a mini USB-b cable to connect the PICKIT 3 to your computer</li></ul></li><li>Use a micro USB cable to connect the OSAVC to your computer for power and a communications port</li></ul><h2 id="setting-up-serial-terminal-coolterm-mobaxterm-putty" tabindex="-1">Setting up Serial Terminal (CoolTerm / mobaxterm / putty) <a class="header-anchor" href="#setting-up-serial-terminal-coolterm-mobaxterm-putty" aria-label="Permalink to &quot;Setting up Serial Terminal (CoolTerm / mobaxterm / putty)&quot;">​</a></h2><ul><li><p>Open CoolTerm after connecting your PC to the OSAVC</p></li><li><p>Choose the port at the bottom left of the application as usbserial <img src="'+l+'" alt="usbSerial"></p></li><li><p>Click Options -&gt; Select 115200 as the baud rate -&gt; Click OK to save <img src="'+r+'" alt="baudrate"></p></li><li><p>Click Connect at the top of the application</p></li></ul><h2 id="setting-up-the-mplab-x" tabindex="-1">Setting up the MPLAB X <a class="header-anchor" href="#setting-up-the-mplab-x" aria-label="Permalink to &quot;Setting up the MPLAB X&quot;">​</a></h2><h3 id="follow-the-below-given-setps-for-running-serial-x" tabindex="-1">Follow the below given setps for running Serial.X <a class="header-anchor" href="#follow-the-below-given-setps-for-running-serial-x" aria-label="Permalink to &quot;Follow the below given setps for running Serial.X&quot;">​</a></h3><ul><li><p>File -&gt; Open Project <img src="'+n+'" alt="Open Project"></p></li><li><p>Now navigate to -&gt; open-source-autonomous-vehicle-controller -&gt; lib -&gt; Serial.X you would be able to see the below window with the serial project open <img src="'+i+'" alt="Serial Project window"></p></li><li><p>Now Open Serial.X Project Properties (File -&gt; Project Properties) <img src="'+p+'" alt="Serial Project Properties"></p></li><li><p>Choose Connected Hardware Tool to PICkit3</p></li><li><p>Navigate to xc32-ld <img src="'+c+'" alt="xc32-id"></p></li><li><p>Choose General as the option category <img src="'+u+'" alt="General option category"></p></li><li><p>set the Heap Size (bytes) to be 8000 bytes <img src="'+h+'" alt="Heap Size"></p></li><li><p>Click OK to save the modified Project Properties <img src="'+m+'" alt="OK"></p></li><li><p>Click Clean and Build <img src="'+d+'" alt="Clean and Build"></p></li><li><p>Click Make and Program Device <img src="'+g+'" alt="Make and Program Device"></p></li><li><p>Open your preset serial terminal application (CoolTerm)</p></li></ul><p>Output should be: <img src="'+b+'" alt="Output from CoolTerm"></p><ul><li><p>Try typing something and press Enter <img src="'+C+'" alt="Something"></p></li><li><p>What you type should now stay in the serial terminal</p></li></ul>',19),v=[_];function f(P,S,k,w,A,j){return o(),t("div",null,v)}const T=e(y,[["render",f]]);export{D as __pageData,T as default};
