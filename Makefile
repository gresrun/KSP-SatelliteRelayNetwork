PLUGINNAME = Satellite\ Relays.dll

KSP = /Applications/KSP_0.14/
DLL = $(KSP)KSP.app/Contents/Data/Managed/
REFS = $(DLL)Assembly-CSharp.dll,$(DLL)UnityEngine.dll
OUTPUT = $(KSP)Plugins/$(PLUGINNAME)
SOURCE = $(wildcard *.cs)


all:
	gmcs -t:library -r:$(REFS) $(SOURCE) -out:$(OUTPUT)
