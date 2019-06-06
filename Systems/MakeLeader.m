function [System, SInfo] = MakeLeader(System_In, SInfo_In, target, B)

    System = System_In;
    System.isLeader = true;
    System.B = B;
    System.target = target;
    SInfo = SInfo_In;
    save(SInfo_In.filename, 'System');
end