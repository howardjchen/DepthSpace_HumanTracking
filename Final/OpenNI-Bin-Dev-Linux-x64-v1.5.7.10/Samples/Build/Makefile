include Common/CommonDefs.mak

.PHONY: all

NETPROJ = 
ifneq "$(realpath /usr/bin/gmcs)" ""
	NETPROJ += SimpleViewer.net
	NETPROJ += SimpleRead.net
	NETPROJ += UserTracker.net
endif

all: $(NETPROJ) SimpleViewer.java NiRecordSynthetic NiConvertXToONI NiSampleModule NiUserSelection NiSimpleCreate NiUserTracker NiBackRecorder UserTracker.java NiSimpleSkeleton NiSimpleRead NiSimpleViewer NiSkeletonBenchmark NiViewer NiAudioSample NiCRead SimpleRead.java NiHandTracker 


.PHONY: SimpleViewer.java
SimpleViewer.java:
	$(MAKE) -C ../SimpleViewer.java

.PHONY: SimpleViewer.net
SimpleViewer.net:
	$(MAKE) -C ../SimpleViewer.net

.PHONY: NiRecordSynthetic
NiRecordSynthetic:
	$(MAKE) -C ../NiRecordSynthetic

.PHONY: NiConvertXToONI
NiConvertXToONI:
	$(MAKE) -C ../NiConvertXToONI

.PHONY: NiSampleModule
NiSampleModule:
	$(MAKE) -C ../NiSampleModule

.PHONY: NiUserSelection
NiUserSelection:
	$(MAKE) -C ../NiUserSelection

.PHONY: NiSimpleCreate
NiSimpleCreate:
	$(MAKE) -C ../NiSimpleCreate

.PHONY: NiUserTracker
NiUserTracker:
	$(MAKE) -C ../NiUserTracker

.PHONY: NiBackRecorder
NiBackRecorder:
	$(MAKE) -C ../NiBackRecorder

.PHONY: UserTracker.java
UserTracker.java:
	$(MAKE) -C ../UserTracker.java

.PHONY: SimpleRead.net
SimpleRead.net:
	$(MAKE) -C ../SimpleRead.net

.PHONY: NiSimpleSkeleton
NiSimpleSkeleton:
	$(MAKE) -C ../NiSimpleSkeleton

.PHONY: NiSimpleRead
NiSimpleRead:
	$(MAKE) -C ../NiSimpleRead

.PHONY: NiSimpleViewer
NiSimpleViewer:
	$(MAKE) -C ../NiSimpleViewer

.PHONY: NiSkeletonBenchmark
NiSkeletonBenchmark:
	$(MAKE) -C ../NiSkeletonBenchmark

.PHONY: UserTracker.net
UserTracker.net:
	$(MAKE) -C ../UserTracker.net

.PHONY: NiViewer
NiViewer:
	$(MAKE) -C ../NiViewer

.PHONY: NiAudioSample
NiAudioSample:
	$(MAKE) -C ../NiAudioSample

.PHONY: NiCRead
NiCRead:
	$(MAKE) -C ../NiCRead

.PHONY: SimpleRead.java
SimpleRead.java:
	$(MAKE) -C ../SimpleRead.java

.PHONY: NiHandTracker
NiHandTracker:
	$(MAKE) -C ../NiHandTracker
