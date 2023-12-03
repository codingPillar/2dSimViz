export class Communication{
    private address: string;
    private port: string;

    constructor(address: string, port: string){
        this.address = address;
        this.port = port;
    }

    public async get<Response>(route: string){
        return await (await fetch(`http://${this.address}:${this.port}/${route}`)).json() as Response;
    }

    public async post<Request, Response>(route: string, body: Request): Promise<Response>{
        /* TODO IMPLEMENT */
        return (await new Promise((resolve, reject) => {
            resolve({} as Response);
        })) as Response;
    }
}